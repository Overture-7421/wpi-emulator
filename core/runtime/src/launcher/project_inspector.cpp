#include "wpiemu/core/launcher/project_inspector.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <regex>

#include "wpiemu/sdk/core/strings.h"

namespace wpiemu::core::launcher {

using namespace wpiemu::sdk;
namespace fs = std::filesystem;

namespace {

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
        const std::string ext = asciiLower(it->path().extension().string());
        for (const char* e : extensions)
            if (ext == e) return true;
    }
    return false;
}

std::string stringOf(const Value* v) { return v != nullptr && v->isString() ? v->asString() : (v != nullptr && v->isNumber() ? std::to_string(v->asInt()) : std::string()); }

}  // namespace

bool ProjectReport::hasVendorDep(std::string_view fragment) const {
    const std::string needle = asciiLower(fragment);
    return std::any_of(vendorDeps.begin(), vendorDeps.end(), [&](const VendorDep& d) {
        return asciiLower(d.name).find(needle) != std::string::npos || asciiLower(d.fileName).find(needle) != std::string::npos;
    });
}

std::string ProjectReport::photonLibVersion() const {
    for (const VendorDep& d : vendorDeps)
        if (!d.version.empty() && (asciiLower(d.name).find("photon") != std::string::npos || asciiLower(d.fileName).find("photon") != std::string::npos))
            return d.version;
    return {};
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
            prefsLanguage = asciiLower(stringOf(prefs.find("currentLanguage")));
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
            if (it->is_regular_file(lec) && asciiLower(it->path().extension().string()) == ".json") files.push_back(it->path());
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

std::vector<fs::path> wpilibRootCandidates(config::Platform platform, const fs::path& publicDir, const fs::path& homeDir) {
    std::vector<fs::path> roots;
    if (platform == config::Platform::Windows) {
        if (!publicDir.empty()) roots.push_back(publicDir / "wpilib");
        roots.push_back(fs::path("C:\\Users\\Public") / "wpilib");
        if (!homeDir.empty()) roots.push_back(homeDir / "wpilib");   // "install for this user only"
    } else if (!homeDir.empty()) {
        roots.push_back(homeDir / "wpilib");
    }
    std::vector<fs::path> unique;
    for (const fs::path& p : roots)
        if (std::find(unique.begin(), unique.end(), p) == unique.end()) unique.push_back(p);
    return unique;
}

std::vector<fs::path> wpilibRootCandidates(config::Platform platform, const sdk::EnvLookup& env) {
    auto folder = [&](std::string_view name) {
        const std::optional<std::string> value = env ? env(name) : std::nullopt;
        return value ? fs::path(*value) : fs::path();
    };
    return wpilibRootCandidates(platform, folder("PUBLIC"), folder(platform == config::Platform::Windows ? "USERPROFILE" : "HOME"));
}

namespace {
fs::path javaBinary(const fs::path& jdk, bool windows) { return jdk / "bin" / (windows ? "java.exe" : "java"); }
}  // namespace

std::optional<WpilibInstall> wpilibInstallAt(const fs::path& home, int year) {
    if (!isDirectory(home)) return std::nullopt;
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

std::optional<WpilibInstall> findWpilibInstall(int year, const std::vector<fs::path>& roots) {
    const std::vector<fs::path> candidates = roots.empty() ? wpilibRootCandidates(config::currentPlatform(), &sdk::readEnvironment) : roots;
    for (const fs::path& root : candidates)
        if (auto install = wpilibInstallAt(root / std::to_string(year), year)) return install;
    return std::nullopt;
}

std::vector<WpilibInstall> listWpilibInstalls(const std::vector<fs::path>& roots) {
    const std::vector<fs::path> candidates = roots.empty() ? wpilibRootCandidates(config::currentPlatform(), &sdk::readEnvironment) : roots;
    std::vector<WpilibInstall> found;
    for (const fs::path& root : candidates) {
        std::error_code ec;
        for (fs::directory_iterator it(root, ec), end; !ec && it != end; it.increment(ec)) {
            const std::string name = it->path().filename().string();
            if (name.size() != 4 || !std::all_of(name.begin(), name.end(), [](unsigned char c) { return std::isdigit(c) != 0; })) continue;
            const int year = std::stoi(name);
            if (std::any_of(found.begin(), found.end(), [&](const WpilibInstall& i) { return i.year == year; })) continue;
            if (auto install = wpilibInstallAt(it->path(), year)) found.push_back(std::move(*install));
        }
    }
    std::sort(found.begin(), found.end(), [](const WpilibInstall& a, const WpilibInstall& b) { return a.year > b.year; });
    return found;
}

}  // namespace wpiemu::core::launcher
