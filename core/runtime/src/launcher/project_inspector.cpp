#include "wpiemu/core/launcher/project_inspector.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <regex>

#include "wpiemu/core/launcher/process.h"

namespace wpiemu::core::launcher {

using namespace wpiemu::sdk;
namespace fs = std::filesystem;

namespace {

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

bool isDirectory(const fs::path& p) {
    std::error_code ec;
    return fs::is_directory(p, ec) && !ec;
}
bool isFile(const fs::path& p) {
    std::error_code ec;
    return fs::is_regular_file(p, ec) && !ec;
}

std::optional<std::string> firstMatch(const std::string& text, const char* pattern) {
    try {
        std::smatch m;
        if (std::regex_search(text, m, std::regex(pattern)) && m.size() > 1) return m[1].str();
    } catch (const std::regex_error&) {
    }
    return std::nullopt;
}

bool contains(const std::string& text, const char* pattern) {
    try {
        return std::regex_search(text, std::regex(pattern));
    } catch (const std::regex_error&) {
        return false;
    }
}

constexpr int kMaxTeamNumber = 99999;  // FRC team numbers have at most five digits

int yearOf(const std::string& version) {
    if (version.size() >= 4 && std::all_of(version.begin(), version.begin() + 4, [](char c) { return std::isdigit(static_cast<unsigned char>(c)) != 0; }))
        return std::stoi(version.substr(0, 4));
    return 0;
}

bool hasSourceFiles(const fs::path& dir, std::initializer_list<const char*> extensions) {
    std::error_code ec;
    if (!isDirectory(dir)) return false;
    for (fs::recursive_directory_iterator it(dir, fs::directory_options::skip_permission_denied, ec), end; !ec && it != end; it.increment(ec)) {
        if (!it->is_regular_file(ec)) continue;
        const std::string ext = lower(it->path().extension().string());
        for (const char* e : extensions)
            if (ext == e) return true;
    }
    return false;
}

std::string stringOf(const Value* v) { return v != nullptr && v->isString() ? v->asString() : (v != nullptr && v->isNumber() ? std::to_string(v->asInt()) : std::string()); }

}  // namespace

bool ProjectReport::hasVendorDep(std::string_view fragment) const {
    const std::string needle = lower(std::string(fragment));
    return std::any_of(vendorDeps.begin(), vendorDeps.end(), [&](const VendorDep& d) {
        return lower(d.name).find(needle) != std::string::npos || lower(d.fileName).find(needle) != std::string::npos;
    });
}

Result<ProjectReport> ProjectInspector::inspect(const fs::path& dir) const {
    std::error_code ec;
    if (dir.empty()) return makeError(ErrorCode::InvalidArgument, "no project directory given");
    if (!fs::exists(dir, ec)) return makeError(ErrorCode::NotFound, "no such directory: " + dir.string());
    if (!isDirectory(dir)) return makeError(ErrorCode::InvalidArgument, "not a directory: " + dir.string());

    ProjectReport r;
    r.root = dir;
    r.name = dir.filename().empty() ? dir.parent_path().filename().string() : dir.filename().string();
    auto note = [&](Severity s, std::string path, std::string message) { r.notes.push_back({s, std::move(path), std::move(message), {}}); };

    // ---- build script ----
    std::string buildText;
    for (const char* name : {"build.gradle", "build.gradle.kts"}) {
        const fs::path p = dir / name;
        if (!isFile(p)) continue;
        r.hasBuildScript = true;
        if (const auto text = config::readTextFile(p)) buildText = text.value();
        else note(Severity::Warning, name, "cannot read: " + text.error().message);
        break;
    }
    if (!r.hasBuildScript) note(Severity::Error, "build.gradle", "no build.gradle or build.gradle.kts in " + dir.string());

    if (const auto v = firstMatch(buildText, R"(edu\.wpi\.first\.GradleRIO["'\)\s]*version\s*["']([^"']+)["'])")) {
        r.gradleRioFound = true;
        r.gradleRioVersion = *v;
    } else if (const auto cp = firstMatch(buildText, R"(edu\.wpi\.first:GradleRIO:([0-9][^"']*)["'])")) {
        r.gradleRioFound = true;
        r.gradleRioVersion = *cp;
    } else if (contains(buildText, R"(edu\.wpi\.first\.GradleRIO)")) {
        r.gradleRioFound = true;  // plugin applied, version declared elsewhere (a variable / settings plugin management)
        note(Severity::Warning, "build.gradle", "GradleRIO plugin found but its version is not a literal; the year comes from the preferences");
    } else if (r.hasBuildScript) {
        note(Severity::Error, "build.gradle", "the GradleRIO plugin (edu.wpi.first.GradleRIO) is not applied: this is not a WPILib project");
    }
    r.wpilibYear = yearOf(r.gradleRioVersion);

    // ---- .wpilib/wpilib_preferences.json ----
    std::string prefsLanguage;
    bool hasPrefs = false;
    if (const auto text = config::readTextFile(dir / ".wpilib" / "wpilib_preferences.json")) {
        hasPrefs = true;
        const Result<Value> parsed = json_.parse(text.value());
        if (parsed && parsed.value().isObject()) {
            const Value& prefs = parsed.value();
            if (const Value* t = prefs.find("teamNumber")) {
                const std::string team = stringOf(t);
                const bool digits = !team.empty() && std::all_of(team.begin(), team.end(), [](char c) { return std::isdigit(static_cast<unsigned char>(c)) != 0; });
                int number = 0;
                const auto parsedTeam = digits ? std::from_chars(team.data(), team.data() + team.size(), number) : std::from_chars_result{team.data(), std::errc::invalid_argument};
                if (parsedTeam.ec == std::errc{} && parsedTeam.ptr == team.data() + team.size() && number >= 0 && number <= kMaxTeamNumber) r.teamNumber = number;
                else if (!team.empty()) note(Severity::Warning, ".wpilib/wpilib_preferences.json", "teamNumber '" + team + "' is not a valid FRC team number (0-" + std::to_string(kMaxTeamNumber) + ")");
            }
            prefsLanguage = lower(stringOf(prefs.find("currentLanguage")));
            if (r.wpilibYear == 0) r.wpilibYear = yearOf(stringOf(prefs.find("projectYear")));
        } else {
            note(Severity::Warning, ".wpilib/wpilib_preferences.json", "not valid JSON: " + (parsed ? std::string("not an object") : parsed.error().message));
        }
    }
    if (r.wpilibYear == 0)
        if (const auto settings = config::readTextFile(dir / "settings.gradle"))
            if (const auto y = firstMatch(settings.value(), R"(frcYear\s*=\s*["'](\d{4})["'])")) r.wpilibYear = std::stoi(*y);
    r.isWpilibProject = r.hasBuildScript && (r.gradleRioFound || (hasPrefs && r.wpilibYear > 0));
    if (r.isWpilibProject && r.wpilibYear == 0) note(Severity::Warning, "build.gradle", "could not determine the WPILib year");

    // ---- language ----
    const bool javaSignal = contains(buildText, R"(frcJava|FRCJavaArtifact|id\s*[\(]?\s*["']java["']|wpi\.java)");
    const bool cppSignal = contains(buildText, R"(frcNative|FRCNativeArtifact|id\s*[\(]?\s*["']cpp["']|wpi\.cpp|nativeUtils|NativeUtils)");
    if (prefsLanguage == "java" || prefsLanguage == "cpp") {
        r.language = prefsLanguage;
    } else if (javaSignal != cppSignal) {
        r.language = javaSignal ? "java" : "cpp";
    } else {
        const bool javaSources = hasSourceFiles(dir / "src" / "main" / "java", {".java"});
        const bool cppSources = hasSourceFiles(dir / "src" / "main" / "cpp", {".cpp", ".cc", ".cxx", ".h", ".hpp"});
        if (javaSources != cppSources) r.language = javaSources ? "java" : "cpp";
    }
    if (r.isWpilibProject && r.language.empty()) note(Severity::Warning, "build.gradle", "could not tell Java from C++");
    if (!prefsLanguage.empty() && (javaSignal != cppSignal) && prefsLanguage != (javaSignal ? "java" : "cpp") && (prefsLanguage == "java" || prefsLanguage == "cpp"))
        note(Severity::Warning, ".wpilib/wpilib_preferences.json", "currentLanguage disagrees with the build script; using the preferences");

    // ---- gradle wrapper ----
    r.hasGradlewScript = isFile(dir / "gradlew");
    r.hasGradlewBat = isFile(dir / "gradlew.bat");
    r.hasWrapperJar = isFile(dir / "gradle" / "wrapper" / "gradle-wrapper.jar");
    if (const auto props = config::readTextFile(dir / "gradle" / "wrapper" / "gradle-wrapper.properties")) {
        r.hasWrapperProperties = true;
        std::string text = props.value();
        text.erase(std::remove(text.begin(), text.end(), '\\'), text.end());  // distributionUrl escapes ':' as '\:'
        if (const auto v = firstMatch(text, R"(gradle-([0-9][0-9A-Za-z.\-]*?)-(?:bin|all)\.zip)")) r.gradleVersion = *v;
    }

    // ---- vendordeps ----
    std::error_code lec;
    if (isDirectory(dir / "vendordeps")) {
        std::vector<fs::path> files;
        for (fs::directory_iterator it(dir / "vendordeps", lec), end; !lec && it != end; it.increment(lec))
            if (it->is_regular_file(lec) && lower(it->path().extension().string()) == ".json") files.push_back(it->path());
        std::sort(files.begin(), files.end());
        for (const fs::path& f : files) {
            VendorDep dep;
            dep.fileName = f.filename().string();
            const auto text = config::readTextFile(f);
            const Result<Value> parsed = text ? json_.parse(text.value()) : Result<Value>(text.error());
            if (parsed && parsed.value().isObject()) {
                const Value& v = parsed.value();
                dep.name = stringOf(v.find("name"));
                dep.version = stringOf(v.find("version"));
                dep.uuid = stringOf(v.find("uuid"));
                dep.frcYear = stringOf(v.find("frcYear"));
            } else {
                dep.readable = false;
                note(Severity::Warning, "vendordeps/" + dep.fileName, "cannot read vendor dependency file");
            }
            if (dep.name.empty()) dep.name = f.stem().string();
            r.vendorDeps.push_back(std::move(dep));
        }
    }
    return r;
}

// ---- WPILib install ------------------------------------------------------------------------------------------------------

std::vector<fs::path> wpilibRootCandidates(config::Platform platform, const config::EnvLookup& env) {
    std::vector<fs::path> roots;
    auto get = [&](std::string_view name) { return env ? env(name) : std::nullopt; };
    if (platform == config::Platform::Windows) {
        if (const auto pub = get("PUBLIC")) roots.push_back(fs::path(*pub) / "wpilib");
        roots.push_back(fs::path("C:\\Users\\Public") / "wpilib");
    } else if (const auto home = get("HOME")) {
        roots.push_back(fs::path(*home) / "wpilib");
    }
    std::vector<fs::path> unique;
    for (const fs::path& p : roots)
        if (std::find(unique.begin(), unique.end(), p) == unique.end()) unique.push_back(p);
    return unique;
}

namespace {
fs::path javaBinary(const fs::path& jdk, bool windows) { return jdk / "bin" / (windows ? "java.exe" : "java"); }
bool windowsPaths(config::Platform p) { return p == config::Platform::Windows; }
}  // namespace

std::optional<WpilibInstall> findWpilibInstall(int year, const std::vector<fs::path>& roots) {
    const std::vector<fs::path> candidates = roots.empty() ? wpilibRootCandidates(config::currentPlatform(), config::systemEnv()) : roots;
    for (const fs::path& root : candidates) {
        const fs::path home = root / std::to_string(year);
        if (!isDirectory(home)) continue;
        WpilibInstall install;
        install.home = home;
        install.year = year;
        const bool windows = config::currentPlatform() == config::Platform::Windows;
        if (isFile(javaBinary(home / "jdk", windows)) || isFile(javaBinary(home / "jdk", !windows))) install.jdkHome = home / "jdk";
        install.hasMaven = isDirectory(home / "maven");
        install.hasTools = isDirectory(home / "tools");
        install.hasRoboRioToolchain = isDirectory(home / "roborio");
        return install;
    }
    return std::nullopt;
}

bool ToolchainReport::ready() const {
    return std::none_of(checks.begin(), checks.end(), [](const ToolchainCheck& c) { return c.status == CheckStatus::Error; });
}

const ToolchainCheck* ToolchainReport::find(std::string_view id) const {
    for (const ToolchainCheck& c : checks)
        if (c.id == id) return &c;
    return nullptr;
}

// ---- toolchain checks --------------------------------------------------------------------------------------------------------

namespace {

std::string javaVersionOf(const fs::path& jdkHome) {
    const auto text = config::readTextFile(jdkHome / "release");
    if (!text) return {};
    const auto v = firstMatch(text.value(), R"re(JAVA_VERSION\s*=\s*"([^"]+)")re");
    return v.value_or(std::string());
}

int javaMajor(const std::string& version) {
    if (version.empty()) return 0;
    try {
        const std::size_t dot = version.find('.');
        const int first = std::stoi(version.substr(0, dot));
        if (first == 1 && dot != std::string::npos) return std::stoi(version.substr(dot + 1));  // "1.8.0_x"
        return first;
    } catch (...) {
        return 0;
    }
}

bool compilerOnPath(const ToolchainContext& ctx, std::initializer_list<const char*> names, std::string& where) {
    const bool windows = windowsPaths(ctx.platform);
    const std::string path = ctx.env ? ctx.env("PATH").value_or("") : std::string();
    for (const char* n : names) {
        for (const fs::path& dir : ctx.hostCompilerDirs)
            if (const auto f = findOnPath(n, dir.string(), windows)) {
                where = f->string();
                return true;
            }
        if (const auto f = findOnPath(n, path, windows)) {
            where = f->string();
            return true;
        }
    }
    return false;
}

bool findMsvc(const ToolchainContext& ctx, std::string& where) {
    for (const char* var : {"ProgramFiles", "ProgramFiles(x86)"}) {
        const auto base = ctx.env ? ctx.env(var) : std::nullopt;
        if (!base) continue;
        const fs::path vs = fs::path(*base) / "Microsoft Visual Studio";
        std::error_code ec;
        if (!isDirectory(vs)) continue;
        for (fs::directory_iterator yearIt(vs, ec), end; !ec && yearIt != end; yearIt.increment(ec)) {
            if (!yearIt->is_directory(ec)) continue;
            std::error_code ec2;
            for (fs::directory_iterator edIt(yearIt->path(), ec2), end2; !ec2 && edIt != end2; edIt.increment(ec2))
                if (isDirectory(edIt->path() / "VC" / "Tools" / "MSVC")) {
                    where = edIt->path().string();
                    return true;
                }
        }
    }
    return false;
}

}  // namespace

ToolchainReport inspectToolchain(const ProjectReport& project, const ToolchainContext& ctx) {
    ToolchainReport report;
    auto add = [&](std::string id, std::string title, CheckStatus status, std::string detail, std::string fix = {}) {
        report.checks.push_back({std::move(id), std::move(title), status, std::move(detail), std::move(fix)});
    };
    const bool windows = windowsPaths(ctx.platform);

    // project + GradleRIO
    if (!project.isWpilibProject) {
        add("project", "WPILib project", CheckStatus::Error, "no GradleRIO build script found in " + project.root.string(),
            "Pick the folder that contains build.gradle with the edu.wpi.first.GradleRIO plugin.");
    } else {
        add("project", "WPILib project", CheckStatus::Ok,
            project.name + " (" + (project.language.empty() ? "language unknown" : project.language == "java" ? "Java" : "C++") + ")");
        if (project.language.empty())
            add("language", "Robot language", CheckStatus::Warning, "could not tell Java from C++", "Open the robot project settings and choose the language.");
    }
    if (!project.gradleRioVersion.empty())
        add("gradlerio", "GradleRIO", CheckStatus::Ok, "version " + project.gradleRioVersion + " (WPILib " + std::to_string(project.wpilibYear) + ")");
    else if (project.isWpilibProject)
        add("gradlerio", "GradleRIO", project.wpilibYear > 0 ? CheckStatus::Warning : CheckStatus::Error,
            project.wpilibYear > 0 ? "version not a literal; year " + std::to_string(project.wpilibYear) + " from the preferences" : "version unknown",
            "Use a literal GradleRIO version in build.gradle.");

    // game plugin for the year
    if (ctx.hasGamePlugin && project.wpilibYear > 0) {
        if (ctx.hasGamePlugin(project.wpilibYear)) add("game_plugin", "Game plugin", CheckStatus::Ok, "a plugin for " + std::to_string(project.wpilibYear) + " is available");
        else
            add("game_plugin", "Game plugin", CheckStatus::Warning, "no plugin for " + std::to_string(project.wpilibYear) + ": the field and game pieces are unavailable (core features only)",
                "Update the emulator or add a plugin for this season.");
    }

    // gradle wrapper
    {
        const bool script = windows ? project.hasGradlewBat : project.hasGradlewScript;
        if (!script)
            add("gradle_wrapper", "Gradle wrapper", CheckStatus::Error, std::string(windows ? "gradlew.bat" : "gradlew") + " is missing from the project",
                "Run `gradle wrapper` in the project or copy the wrapper from a fresh WPILib project.");
        else if (!project.hasWrapperJar || !project.hasWrapperProperties)
            add("gradle_wrapper", "Gradle wrapper", CheckStatus::Warning, "gradle/wrapper is incomplete", "Re-create the wrapper (`gradle wrapper`).");
        else
            add("gradle_wrapper", "Gradle wrapper", CheckStatus::Ok, project.gradleVersion.empty() ? "present" : "Gradle " + project.gradleVersion);
    }

    // WPILib installation
    if (project.wpilibYear > 0) {
        report.wpilib = findWpilibInstall(project.wpilibYear, ctx.wpilibRoots.empty() ? wpilibRootCandidates(ctx.platform, ctx.env) : ctx.wpilibRoots);
        if (report.wpilib)
            add("wpilib_install", "WPILib installation", CheckStatus::Ok,
                report.wpilib->home.string() + (report.wpilib->hasMaven ? " (offline maven present)" : " (no offline maven: Gradle will download dependencies)"));
        else
            add("wpilib_install", "WPILib installation", CheckStatus::Warning, "WPILib " + std::to_string(project.wpilibYear) + " is not installed in the default location",
                "Install the WPILib " + std::to_string(project.wpilibYear) + " suite (it provides the JDK and the offline Maven repository).");
    }

    // JDK (Gradle needs one for Java and C++ projects alike)
    {
        fs::path jdk;
        std::string source;
        if (report.wpilib && !report.wpilib->jdkHome.empty()) {
            jdk = report.wpilib->jdkHome;
            source = "WPILib JDK";
        } else if (const auto home = ctx.env ? ctx.env("JAVA_HOME") : std::nullopt; home && (isFile(javaBinary(*home, windows)))) {
            jdk = *home;
            source = "JAVA_HOME";
        } else if (const auto onPath = findOnPath("java", ctx.env ? ctx.env("PATH").value_or("") : "", windows)) {
            source = "java on PATH";
            jdk = fs::path();
            report.javaVersion.clear();
            add("jdk", "Java (JDK)", CheckStatus::Warning, "java found on PATH (" + onPath->string() + "), version unknown",
                "Install the WPILib suite, which ships the JDK Gradle needs.");
        }
        if (!jdk.empty()) {
            report.jdkHome = jdk;
            report.javaVersion = javaVersionOf(jdk);
            const int major = javaMajor(report.javaVersion);
            if (major != 0 && major < 17)
                add("jdk", "Java (JDK)", CheckStatus::Warning, source + " is Java " + report.javaVersion + "; WPILib needs 17 or newer", "Use the JDK of the WPILib installation.");
            else
                add("jdk", "Java (JDK)", CheckStatus::Ok, source + (report.javaVersion.empty() ? "" : " " + report.javaVersion) + " at " + jdk.string());
        } else if (source.empty()) {
            add("jdk", "Java (JDK)", CheckStatus::Error, "no JDK found (WPILib install, JAVA_HOME or PATH)", "Install the WPILib suite or set JAVA_HOME to a JDK 17+.");
        }
    }

    // host C++ toolchain
    if (project.language == "cpp") {
        std::string where;
        bool found = false;
        switch (ctx.platform) {
            case config::Platform::Windows: found = compilerOnPath(ctx, {"cl"}, where) || findMsvc(ctx, where); break;
            case config::Platform::MacOS: found = compilerOnPath(ctx, {"clang++", "c++"}, where) || isFile("/usr/bin/clang++"); break;
            case config::Platform::Linux: found = compilerOnPath(ctx, {"g++", "clang++"}, where); break;
        }
        if (found) add("cpp_toolchain", "C++ toolchain", CheckStatus::Ok, where.empty() ? "host compiler found" : where);
        else
            add("cpp_toolchain", "C++ toolchain", CheckStatus::Error, "no host C++ compiler found for simulateNative",
                windows ? "Install Visual Studio Build Tools (Desktop development with C++)." : ctx.platform == config::Platform::MacOS ? "Install the Xcode command line tools (xcode-select --install)." : "Install g++ (e.g. sudo apt install g++).");
    }

    // vendordeps
    if (!project.vendorDeps.empty()) {
        std::string list;
        for (const VendorDep& d : project.vendorDeps) list += (list.empty() ? "" : ", ") + d.name + (d.version.empty() ? "" : " " + d.version);
        CheckStatus st = CheckStatus::Info;
        std::string detail = list;
        for (const VendorDep& d : project.vendorDeps)
            if (!d.frcYear.empty() && project.wpilibYear > 0 && d.frcYear != std::to_string(project.wpilibYear)) {
                st = CheckStatus::Warning;
                detail += "; " + d.name + " targets " + d.frcYear;
            }
        if (project.hasVendorDep("phoenix6") || project.hasVendorDep("phoenix 6")) detail += " (Phoenix 6 detected: TalonFX / CANcoder / Pigeon 2 are simulated)";
        add("vendordeps", "Vendor libraries", st, detail, st == CheckStatus::Warning ? "Update the vendor libraries to the project's WPILib year." : std::string());
    }

    // team number
    if (project.teamNumber >= 0) add("team", "Team number", CheckStatus::Info, std::to_string(project.teamNumber));
    else add("team", "Team number", CheckStatus::Info, "not set in .wpilib/wpilib_preferences.json (not needed for simulation)");

    // the emulator's own extension
    if (!ctx.extensionLibrary.empty()) {
        if (isFile(ctx.extensionLibrary)) add("extension", "Emulator HALSim extension", CheckStatus::Ok, ctx.extensionLibrary.string());
        else add("extension", "Emulator HALSim extension", CheckStatus::Error, "not found: " + ctx.extensionLibrary.string(), "Reinstall the emulator or rebuild halsim_wpiemu.");
    }
    return report;
}

}  // namespace wpiemu::core::launcher
