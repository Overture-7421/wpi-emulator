#include <doctest/doctest.h>

#include "launcher_test_support.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/core/launcher/init_script.h"
#include "wpiemu/sdk/testing/environment.h"

// Launch planning: the generated Gradle init script, the environment contract, and how a request becomes a Gradle or an
// external-direct command line.

using namespace wpiemu::sdk;
using namespace wpiemu::core::launcher;
using wpiemu::core::config::JsonValueCodec;
using wpiemu::sdk::testing::envOf;
using wpiemu::test_runtime::ScratchDir;
using wpiemu::test_runtime::makeRequest;
namespace fs = std::filesystem;

TEST_CASE("init script: ADR 0003 content, no machine specific paths") {
    const std::string s = generateInitScript();
    CHECK(s == generateInitScript());  // deterministic: identical for every run
    CHECK(s.find("WPIEMU_EXT") != std::string::npos);
    CHECK(s.find("HALSIM_EXTENSIONS") != std::string::npos);
    CHECK(s.find("defaultEnabled.set(false)") != std::string::npos);
    CHECK(s.find("JavaSimulationTask") != std::string::npos);
    CHECK(s.find("NativeSimulationTask") != std::string::npos);
    CHECK(s.find("org.wpilib.gradlerio.simulation.NativeSimulationTask") != std::string::npos);
    CHECK(s.find("p.plugins.hasPlugin('org.wpilib.GradleRIO')") != std::string::npos);
    CHECK(s.find("org.gradle.api.tasks.JavaExec") != std::string::npos);
    CHECK(s.find("wpi.sim.environment.put('HALSIM_EXTENSIONS', emuExt)") != std::string::npos);
    CHECK(s.find("entry.classpath = run.get().classpath.files") != std::string::npos);
    CHECK(s.find("t.dependsOn(run.map") == std::string::npos);
    CHECK(s.find("t.dependsOn(p.configurations.getByName('runtimeClasspath'), p.tasks.named('classes'))") != std::string::npos);
    CHECK(s.find("C:\\") == std::string::npos);
    CHECK(s.find("/home/") == std::string::npos);
}

TEST_CASE("launch env: contract variables and overrides") {
    ScratchDir dir;
    LaunchRequest r = makeRequest(dir);
    r.javaHome = "/opt/jdk";
    r.extraEnv = {{"CUSTOM", "1"}, {"WPIEMU_CLOCK", "override"}};
    CHECK(halExtensionsValue(r, true) == r.extensionLibrary.string());
    const auto env = emulatorEnvironment(r, false);
    CHECK(env.at(env_var::kExtension) == r.extensionLibrary.string());
    CHECK(env.at(env_names::kIpcEndpoint) == r.ipcEndpoint);
    CHECK(env.at(env_names::kIpcEndpointAlias) == r.ipcEndpoint);
    CHECK(env.at("JAVA_HOME") == fs::path("/opt/jdk").string());
    CHECK(env.at("CUSTOM") == "1");
    CHECK(env.at(env_names::kClock) == "override");  // extraEnv wins
    CHECK(env.at(env_names::kProgramName) == "robot");
    r.extraEnv.clear();
    r.realtimeClock = true;
    r.javaHome.clear();
    const auto plain = emulatorEnvironment(r, true);
    CHECK(plain.at(env_names::kClock) == "realtime");
    CHECK(plain.count("JAVA_HOME") == 0);
}

TEST_CASE("gradle launch plan: wrapper, flags, init script written to the scratch dir") {
    ScratchDir dir;
    LaunchRequest r = makeRequest(dir);
    r.gradleOffline = true;
    r.extraGradleArgs = {"-Pfoo=bar"};
    for (const bool windows : {true, false}) {
        const auto plan = planGradleLaunch(r, windows);
        REQUIRE_MESSAGE(plan.ok(), (plan.ok() ? "" : plan.error().message));
        const ProcessSpec& s = plan.value();
        CHECK(fs::path(s.program).filename() == (windows ? "gradlew.bat" : "gradlew"));
        CHECK(s.workingDir == r.projectDir);
        const std::vector<std::string> expected = {"--no-daemon", "--console=plain", "--offline", "-I", (r.scratchDir / "wpiemu-init.gradle").string(), "-Pfoo=bar", "simulateJava"};
        CHECK(s.args == expected);
        CHECK(s.env.at(env_var::kExtension) == r.extensionLibrary.string());
        CHECK(s.label == "gradlew simulateJava");
    }
    CHECK(ScratchDir::read(r.scratchDir / "wpiemu-init.gradle") == generateInitScript());

    r.language = "cpp";
    CHECK(planGradleLaunch(r, false).value().args.back() == "simulateNative");
    r.language = "java";
    r.launchProfile = std::string(kLaunchModern);
    CHECK(gradleTaskName(r) == "run");
    CHECK(planGradleLaunch(r, false).value().args.back() == "run");
    r.language = "cpp";
    CHECK(gradleTaskName(r) == "simulateNative");
}

TEST_CASE("gradle launch plan: failure modes") {
    ScratchDir dir;
    LaunchRequest ok = makeRequest(dir);
    {
        LaunchRequest r = ok;
        r.projectDir = dir.path() / "nowhere";
        CHECK(planGradleLaunch(r, false).error().code == ErrorCode::NotFound);
    }
    {
        LaunchRequest r = ok;
        r.extensionLibrary = dir.path() / "missing.dll";
        CHECK(planGradleLaunch(r, false).error().code == ErrorCode::NotFound);
    }
    {
        LaunchRequest r = ok;
        r.extensionLibrary.clear();
        CHECK(planGradleLaunch(r, false).error().code == ErrorCode::InvalidArgument);
    }
    {
        LaunchRequest r = ok;
        r.ipcEndpoint.clear();
        CHECK(planGradleLaunch(r, false).error().code == ErrorCode::InvalidArgument);
    }
    {
        LaunchRequest r = ok;
        r.language = "kotlin";
        CHECK(planGradleLaunch(r, false).error().code == ErrorCode::InvalidArgument);
    }
    {
        LaunchRequest r = ok;
        r.scratchDir.clear();
        CHECK(planGradleLaunch(r, false).error().code == ErrorCode::InvalidArgument);
    }
    {
        LaunchRequest r = ok;
        r.projectDir.clear();
        CHECK(planGradleLaunch(r, false).error().code == ErrorCode::InvalidArgument);
    }
    {
        LaunchRequest r = ok;
        dir.write("blocker", "x");
        r.scratchDir = dir.path() / "blocker";  // a file where the directory should be
        CHECK_FALSE(planGradleLaunch(r, false).ok());
    }
}

TEST_CASE("gradle launch plan: paths gradlew.bat cannot handle launch the wrapper jar directly on Windows") {
    ScratchDir dir;
    for (const char* folder : {"R&D", "Team 1&2", "a^b", "Plain Dir (x86)"}) {
        CAPTURE(folder);
        LaunchRequest r = makeRequest(dir);
        r.projectDir = dir.path() / folder / "robot";
        fs::create_directories(r.projectDir);
        dir.write(std::string(folder) + "/robot/gradlew.bat", "@echo off\r\n");
        dir.write(std::string(folder) + "/robot/gradlew", "#!/bin/sh\n");
        r.gradleOffline = false;
        const fs::path jar = r.projectDir / "gradle" / "wrapper" / "gradle-wrapper.jar";
        dir.write(std::string(folder) + "/robot/gradle/wrapper/gradle-wrapper.jar", "x");
        r.javaHome = dir.path() / "jdk";
        const bool special = std::string_view(folder).find_first_of("&^") != std::string_view::npos;

        const auto plan = planGradleLaunch(r, true);
        REQUIRE_MESSAGE(plan.ok(), (plan.ok() ? "" : plan.error().message));
        const ProcessSpec& s = plan.value();
        if (special) {
            CHECK(s.program == (r.javaHome / "bin" / "java.exe").string());
            const std::vector<std::string> head = {"-Xmx64m", "-Xms64m", "-Dorg.gradle.appname=gradlew", "-classpath", jar.string(),
                                                   "org.gradle.wrapper.GradleWrapperMain", "--no-daemon", "--console=plain",
                                                   "-Dorg.gradle.java.home=" + r.javaHome.string(), "-I"};
            REQUIRE(s.args.size() > head.size());
            CHECK(std::vector<std::string>(s.args.begin(), s.args.begin() + static_cast<std::ptrdiff_t>(head.size())) == head);
            CHECK(s.args.back() == "simulateJava");
        } else {
            CHECK(fs::path(s.program).filename() == "gradlew.bat");
            CHECK(s.args.front() == "--no-daemon");
        }
        CHECK(s.workingDir == r.projectDir);

        const auto describe = planExternalDescribe(r, true);
        REQUIRE(describe.ok());
        CHECK((fs::path(describe.value().program).filename() == "gradlew.bat") == !special);
        CHECK(describe.value().args.back() == "simulateExternalJavaRelease");

        // non-Windows plans are never rewritten, and a missing wrapper jar is reported instead of falling back to a broken script
        CHECK(fs::path(planGradleLaunch(r, false).value().program).filename() == "gradlew");
        if (special) {
            fs::remove(jar);
            const auto missing = planGradleLaunch(r, true);
            REQUIRE_FALSE(missing.ok());
            CHECK(missing.error().code == ErrorCode::NotFound);
        }
    }
}

TEST_CASE("external launch plan: describe task names and descriptor paths") {
    ScratchDir dir;
    LaunchRequest r = makeRequest(dir, LaunchMode::ExternalDirect);
    CHECK(externalTaskName(r) == "simulateExternalJavaRelease");
    CHECK(externalDescriptorPath(r) == r.projectDir / "build" / "sim" / "release_java.json");
    r.launchProfile = std::string(kLaunchModern);
    CHECK(externalTaskName(r) == "simulateExternalJava");
    CHECK(externalDescriptorPath(r) == r.projectDir / "build" / "sim" / "java.json");
    const auto modernDescribe = planExternalDescribe(r, false);
    REQUIRE(modernDescribe.ok());
    CHECK(modernDescribe.value().args.back() == "simulateExternalJava");
    CHECK(modernDescribe.value().env.at(env_var::kExtension) == r.extensionLibrary.string());
    CHECK(std::find(modernDescribe.value().args.begin(), modernDescribe.value().args.end(), "-I") != modernDescribe.value().args.end());
    CHECK(ScratchDir::read(r.scratchDir / "wpiemu-init.gradle") == generateInitScript());
    LaunchRequest noScratch = r;
    noScratch.scratchDir.clear();
    CHECK(planExternalDescribe(noScratch, false).error().code == ErrorCode::InvalidArgument);
    r.release = false;
    r.language = "cpp";
    CHECK(externalTaskName(r) == "simulateExternalNativeDebug");
    CHECK(externalDescriptorPath(r) == r.projectDir / "build" / "sim" / "debug_native.json");
    r.javaHome = "/jdk";
    const auto plan = planExternalDescribe(r, false);
    REQUIRE(plan.ok());
    CHECK(plan.value().args.back() == "simulateExternalNativeDebug");
    CHECK(plan.value().args.front() == "--no-daemon");
    CHECK(plan.value().args[2] == "-Dorg.gradle.java.home=" + fs::path("/jdk").string());   // wins over the project's gradle.properties
    CHECK(plan.value().env.at("JAVA_HOME") == "/jdk");
    CHECK(plan.value().env.count(env_names::kIpcEndpoint) == 0);  // the build step needs no emulator variables
    r.projectDir = dir.path() / "none";
    CHECK(planExternalDescribe(r, false).error().code == ErrorCode::NotFound);
}

TEST_CASE("external launch plan: Java descriptor -> JVM command line") {
    ScratchDir dir;
    LaunchRequest r = makeRequest(dir, LaunchMode::ExternalDirect);
    dir.write("robot/build/libs/robot-sources.jar", "tiny");
    dir.write("robot/build/libs/robot-plain.jar", "tiny");
    dir.write("robot/build/libs/small.jar", "12345");
    const fs::path fat = dir.write("robot/build/libs/robot.jar", std::string(5000, 'x'));
    const std::string json = R"json({"libraryDir": "C:\\jni\\release", "mainClassName": "frc.robot.Main",
        "environment": {"HALSIM_EXTENSIONS": "C:\\sim\\halsim_gui.dll", "KEEP": "me", "NUM": 5}, "extensions": []})json";
    const JsonValueCodec codec;
    const auto envNone = envOf({});

    {
        const auto plan = planExternalLaunch(r, json, codec, envNone, true, false);
        REQUIRE_MESSAGE(plan.ok(), (plan.ok() ? "" : plan.error().message));
        const ProcessSpec& s = plan.value();
        CHECK(s.program == "java");  // no JAVA_HOME anywhere: rely on PATH
        REQUIRE(s.args.size() == 3);
        CHECK(s.args[0] == "-Djava.library.path=C:\\jni\\release");
        CHECK(s.args[1] == "-jar");
        std::error_code same;
        CHECK(fs::equivalent(fs::path(s.args[2]), fat, same));  // the biggest non-sources jar
        CHECK(s.workingDir == r.projectDir);
        CHECK(s.env.at("HALSIM_EXTENSIONS") == r.extensionLibrary.string());  // ours replaces the stock Sim GUI
        CHECK(s.env.at("KEEP") == "me");
        CHECK(s.env.count("NUM") == 0);  // non-string values are ignored
        CHECK(s.env.at(env_names::kIpcEndpoint) == r.ipcEndpoint);
        CHECK(s.env.at("PATH").rfind("C:\\jni\\release", 0) == 0);  // library dir first
    }
    {  // JAVA_HOME from the host, macOS flag, DYLD path, host value appended
        const auto host = envOf({{"JAVA_HOME", "/jdk17"}, {"DYLD_LIBRARY_PATH", "/existing"}});
        const auto plan = planExternalLaunch(r, R"json({"libraryDir": "/jni"})json", codec, host, false, true);
        REQUIRE(plan.ok());
        const ProcessSpec& s = plan.value();
        CHECK(s.program == (fs::path("/jdk17") / "bin" / "java").string());
        CHECK(s.args.front() == "-XstartOnFirstThread");
        CHECK(s.env.at("DYLD_LIBRARY_PATH") == "/jni:/existing");
    }
    {  // the request's JDK beats the environment; Linux uses LD_LIBRARY_PATH
        r.javaHome = "/wpilib/jdk";
        const auto plan = planExternalLaunch(r, R"json({"libraryDir": "/jni"})json", codec, envNone, false, false);
        REQUIRE(plan.ok());
        CHECK(plan.value().program == (fs::path("/wpilib/jdk") / "bin" / "java").string());
        CHECK(plan.value().env.at("LD_LIBRARY_PATH") == "/jni");
        CHECK(plan.value().env.at("JAVA_HOME") == "/wpilib/jdk");
        CHECK(planExternalLaunch(r, R"json({"libraryDir": "C:\\x"})json", codec, envNone, true, false).value().program.find("java.exe") != std::string::npos);
    }
    {  // GradleRIO writes a LIST with one entry per simulation target (real FRC project); the matching entry is used
        const std::string list = R"json([{"type": "native", "launchfile": "x.exe"},
            {"type": "java", "name": "jar (in project p)", "libraryDir": "/jni", "mainClassName": "frc.robot.Main", "extensions": []}])json";
        const auto plan = planExternalLaunch(r, list, codec, envNone, false, false);
        REQUIRE_MESSAGE(plan.ok(), (plan.ok() ? "" : plan.error().message));
        CHECK(plan.value().args.front() == "-Djava.library.path=/jni");
        CHECK(planExternalLaunch(r, "[]", codec, envNone, false, false).error().code == ErrorCode::ParseError);
    }
    {  // failure modes
        CHECK(planExternalLaunch(r, "{not json", codec, envNone, true, false).error().code == ErrorCode::ParseError);
        CHECK(planExternalLaunch(r, "[1]", codec, envNone, true, false).error().code == ErrorCode::ParseError);
        CHECK(planExternalLaunch(r, "{}", codec, envNone, true, false).error().code == ErrorCode::InvalidArgument);  // no libraryDir
        LaunchRequest noJar = r;
        noJar.projectDir = dir.path() / "empty";
        fs::create_directories(noJar.projectDir / "build" / "libs");
        CHECK(planExternalLaunch(noJar, json, codec, envNone, true, false).error().code == ErrorCode::NotFound);
    }
}

TEST_CASE("external launch plan: a classpath-launch profile uses the descriptor runtime classpath and main class") {
    ScratchDir dir;
    LaunchRequest r = makeRequest(dir, LaunchMode::ExternalDirect);
    r.launchProfile = std::string(kLaunchModern);
    const JsonValueCodec codec;
    const auto host = envOf({});
    const std::string descriptor = R"json([{
        "type": "java", "libraryDir": "/jni", "mainClassName": "frc.robot.Main",
        "classpath": ["build/classes/java/main", "build/resources/main", "/cached libraries/wpilibj.jar"],
        "jvmArgs": ["--enable-native-access=ALL-UNNAMED", "--add-opens", "java.base/java.lang=ALL-UNNAMED"],
        "environment": {"HALSIM_EXTENSIONS": "stock-gui", "KEEP": "1"}
    }])json";
    for (const bool windows : {false, true}) {
        const auto plan = planExternalLaunch(r, descriptor, codec, host, windows, false);
        REQUIRE_MESSAGE(plan.ok(), (plan.ok() ? "" : plan.error().message));
        const auto& spec = plan.value();
        const std::string classpath = windows
                                         ? "build/classes/java/main;build/resources/main;/cached libraries/wpilibj.jar"
                                         : "build/classes/java/main:build/resources/main:/cached libraries/wpilibj.jar";
        const std::vector<std::string> expected = {
            "--enable-native-access=ALL-UNNAMED", "--add-opens", "java.base/java.lang=ALL-UNNAMED",
            "-Djava.library.path=/jni", "-classpath", classpath, "frc.robot.Main"};
        CHECK(spec.args == expected);
        CHECK(spec.workingDir == r.projectDir);
        CHECK(spec.env.at(env_var::kHalExtensions) == r.extensionLibrary.string());
        CHECK(spec.env.at("KEEP") == "1");
    }
    const std::string macDescriptor = R"json({"libraryDir":"/jni", "mainClassName":"Main",
        "classpath":["robot.jar"], "jvmArgs":["-XstartOnFirstThread"]})json";
    const auto macPlan = planExternalLaunch(r, macDescriptor, codec, host, false, true);
    REQUIRE(macPlan.ok());
    CHECK(std::count(macPlan.value().args.begin(), macPlan.value().args.end(), "-XstartOnFirstThread") == 1);

    for (const char* malformed : {
             R"json({"libraryDir":"/jni", "mainClassName":"Main"})json",
             R"json({"libraryDir":"/jni", "mainClassName":"Main", "classpath":[]})json",
             R"json({"libraryDir":"/jni", "mainClassName":"Main", "classpath":[""]})json",
             R"json({"libraryDir":"/jni", "mainClassName":"Main", "classpath":[5]})json",
             R"json({"libraryDir":"/jni", "classpath":["robot.jar"]})json",
             R"json({"libraryDir":"/jni", "mainClassName":"Main", "classpath":["robot.jar"], "jvmArgs":[5]})json"}) {
        const auto plan = planExternalLaunch(r, malformed, codec, host, false, false);
        REQUIRE_FALSE(plan.ok());
        CHECK(plan.error().code == ErrorCode::InvalidArgument);
    }
}

TEST_CASE("external launch plan: native descriptor -> the exe") {
    ScratchDir dir;
    LaunchRequest r = makeRequest(dir, LaunchMode::ExternalDirect, "cpp");
    const JsonValueCodec codec;
    const auto host = envOf({{"PATH", "C:\\Windows"}});
    const std::string json = R"json({"launchfile": "build/install/frcUserProgram/windowsx86-64/release/lib/frcUserProgram.exe",
        "libpaths": ["C:\\libs\\a", "C:\\libs\\b"], "environment": {"X": "1"}})json";
    const auto plan = planExternalLaunch(r, json, codec, host, true, false);
    REQUIRE_MESSAGE(plan.ok(), (plan.ok() ? "" : plan.error().message));
    const ProcessSpec& s = plan.value();
    CHECK(s.program == (r.projectDir / "build/install/frcUserProgram/windowsx86-64/release/lib/frcUserProgram.exe").string());  // relative to the project
    CHECK(s.args.empty());
    CHECK(s.workingDir == r.projectDir);
    CHECK(s.env.at("PATH") == "C:\\libs\\a;C:\\libs\\b;C:\\Windows");
    CHECK(s.env.at("X") == "1");
    CHECK(s.env.at(env_var::kHalExtensions) == r.extensionLibrary.string());

    const fs::path elsewhere = dir.path() / "elsewhere" / "robot";
    std::string escaped;
    for (const char c : elsewhere.string()) escaped += c == '\\' ? std::string("\\\\") : std::string(1, c);
    const auto absolute = planExternalLaunch(r, "{\"launchfile\": \"" + escaped + "\"}", codec, host, false, false);
    REQUIRE(absolute.ok());
    CHECK(absolute.value().program == elsewhere.string());  // already absolute: not re-based on the project
    CHECK(planExternalLaunch(r, "{}", codec, host, true, false).error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("launch profiles: classic is the default, modern runs the project's run task, unknown ids are refused") {
    CHECK(findLaunchProfile("") == findLaunchProfile(kLaunchClassic));
    REQUIRE(findLaunchProfile(kLaunchModern) != nullptr);
    CHECK(findLaunchProfile(kLaunchModern)->javaClasspathLaunch);
    CHECK_FALSE(findLaunchProfile(kLaunchClassic)->javaClasspathLaunch);
    CHECK(findLaunchProfile("gradlerio-unknown") == nullptr);
    ScratchDir dir;
    LaunchRequest r = makeRequest(dir);
    r.launchProfile = "gradlerio-unknown";
    CHECK(planGradleLaunch(r, false).error().code == ErrorCode::InvalidArgument);
}