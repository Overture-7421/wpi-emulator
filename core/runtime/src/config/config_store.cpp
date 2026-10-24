#include "wpiemu/core/config/config_store.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <sstream>
#include <system_error>
#include <thread>

#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/sdk/core/strings.h"

#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

namespace wpiemu::core::config {

using namespace wpiemu::sdk;
namespace fs = std::filesystem;

Platform currentPlatform() {
#if defined(_WIN32)
    return Platform::Windows;
#elif defined(__APPLE__)
    return Platform::MacOS;
#else
    return Platform::Linux;
#endif
}

Result<fs::path> appDataRoot(Platform platform, const EnvLookup& env) {
    auto get = [&](std::string_view name) { return env ? env(name) : std::nullopt; };
    switch (platform) {
        case Platform::Windows: {
            if (const auto appData = get("APPDATA")) return fs::path(*appData) / "WpiEmulator";
            if (const auto profile = get("USERPROFILE")) return fs::path(*profile) / "AppData" / "Roaming" / "WpiEmulator";
            if (const auto local = get("LOCALAPPDATA")) return fs::path(*local) / "WpiEmulator";
            break;
        }
        case Platform::MacOS: {
            if (const auto home = get("HOME")) return fs::path(*home) / "Library" / "Application Support" / "WpiEmulator";
            break;
        }
        case Platform::Linux: {
            if (const auto xdg = get("XDG_DATA_HOME")) return fs::path(*xdg) / "wpi-emulator";
            if (const auto home = get("HOME")) return fs::path(*home) / ".local" / "share" / "wpi-emulator";
            break;
        }
    }
    return makeError(ErrorCode::NotFound, "no home/app-data directory in the environment");
}

Result<fs::path> localDataRoot(Platform platform, const EnvLookup& env) {
    if (platform == Platform::Windows) {
        auto get = [&](std::string_view name) { return env ? env(name) : std::nullopt; };
        if (const auto local = get("LOCALAPPDATA")) return fs::path(*local) / "WpiEmulator";
        if (const auto profile = get("USERPROFILE")) return fs::path(*profile) / "AppData" / "Local" / "WpiEmulator";
        return makeError(ErrorCode::NotFound, "no home/app-data directory in the environment");
    }
    return appDataRoot(platform, env);
}

std::string projectKeyForPath(std::string_view canonicalPath, bool caseInsensitive) {
    std::string path(canonicalPath);
    while (path.size() > 1 && (path.back() == '/' || path.back() == '\\')) path.pop_back();
    std::string hashed = path;
    for (char& c : hashed)
        if (c == '\\') c = '/';
    if (caseInsensitive) hashed = asciiLower(hashed);
    std::uint64_t h = 1469598103934665603ull;  // FNV-1a 64
    for (const char c : hashed) {
        h ^= static_cast<unsigned char>(c);
        h *= 1099511628211ull;
    }
    std::string name;
    const std::size_t slash = hashed.find_last_of('/');
    const std::string folder = slash == std::string::npos ? hashed : hashed.substr(slash + 1);
    for (const char c : folder) {
        if (name.size() >= 32) break;
        name.push_back(std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '-' || c == '_' ? c : '_');
    }
    if (name.empty()) name = "project";
    static constexpr char kHex[] = "0123456789abcdef";
    std::string hex(16, '0');
    for (int i = 15; i >= 0; --i) {
        hex[static_cast<std::size_t>(i)] = kHex[h & 0xF];
        h >>= 4;
    }
    return name + "-" + hex;
}

std::string projectKey(const fs::path& projectDir) {
    std::error_code ec;
    fs::path canonical = fs::weakly_canonical(projectDir, ec);
    if (ec) canonical = fs::absolute(projectDir, ec);
    return projectKeyForPath(canonical.generic_string(), currentPlatform() == Platform::Windows);
}

fs::path appDataConfigPath(const fs::path& root, const fs::path& projectDir) {
    return root / "projects" / projectKey(projectDir) / "robot.json";
}

fs::path projectConfigPath(const fs::path& projectDir) { return projectDir / ".wpiemu" / "robot.json"; }

// ---- files ---------------------------------------------------------------------------------------------

namespace {

std::FILE* openForWrite(const fs::path& p) {
#ifdef _WIN32
    std::FILE* f = nullptr;
    return _wfopen_s(&f, p.c_str(), L"wb") == 0 ? f : nullptr;
#else
    return std::fopen(p.c_str(), "wb");
#endif
}

bool flushToDisk(std::FILE* f) {
    if (std::fflush(f) != 0) return false;
#ifdef _WIN32
    return _commit(_fileno(f)) == 0;
#else
    return fsync(fileno(f)) == 0;
#endif
}

std::string randomSuffix() {
    static thread_local std::mt19937_64 gen{std::random_device{}()};
    std::ostringstream out;
    out << std::hex << (gen() & 0xFFFFFFFFull);
    return out.str();
}

}  // namespace

Status atomicWriteFile(const fs::path& target, std::string_view content) {
    std::error_code ec;
    if (target.has_parent_path()) {
        fs::create_directories(target.parent_path(), ec);
        if (ec) return makeError(ErrorCode::IoError, "cannot create " + target.parent_path().string() + ": " + ec.message());
    }
    const fs::path tmp = target.parent_path() / (target.filename().string() + "." + randomSuffix() + ".tmp");
    std::FILE* f = openForWrite(tmp);
    if (f == nullptr) return makeError(ErrorCode::IoError, "cannot open " + tmp.string() + " for writing");
    const bool wrote = content.empty() || std::fwrite(content.data(), 1, content.size(), f) == content.size();
    const bool flushed = wrote && flushToDisk(f);
    const bool closed = std::fclose(f) == 0;
    if (!(wrote && flushed && closed)) {
        fs::remove(tmp, ec);
        return makeError(ErrorCode::IoError, "failed writing " + tmp.string());
    }
    for (int attempt = 0;; ++attempt) {
        fs::rename(tmp, target, ec);
        if (!ec) return okStatus();
        if (attempt >= 4) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));  // virus scanners / indexers hold files briefly on Windows
    }
    const std::error_code replaceError = ec;
    std::error_code cleanupError;
    fs::remove(tmp, cleanupError);
    return makeError(ErrorCode::IoError, "cannot replace " + target.string() + ": " + replaceError.message());
}

Result<std::string> readTextFile(const fs::path& file) {
    std::error_code ec;
    if (!fs::exists(file, ec) || ec) return makeError(ErrorCode::NotFound, "no such file: " + file.string());
    std::ifstream in(file, std::ios::binary);
    if (!in) return makeError(ErrorCode::IoError, "cannot open " + file.string());
    std::ostringstream buf;
    buf << in.rdbuf();
    if (in.bad()) return makeError(ErrorCode::IoError, "read error on " + file.string());
    return buf.str();
}

// ---- FileConfigStore -----------------------------------------------------------------------------------

FileConfigStore::FileConfigStore(fs::path file, std::string label, const IValueTextCodec& text, const RobotConfigCodec& codec)
    : file_(std::move(file)), label_(std::move(label)), text_(text), codec_(codec) {}

bool FileConfigStore::exists() const {
    std::error_code ec;
    return fs::exists(file_, ec) && !ec;
}

Result<fs::path> FileConfigStore::backup(std::string_view tag) const {
    std::error_code ec;
    if (!exists()) return makeError(ErrorCode::NotFound, "nothing to back up: " + file_.string());
    fs::path candidate = file_;
    candidate += "." + std::string(tag) + ".bak";
    for (int n = 2; fs::exists(candidate, ec) && n < 1000; ++n) {
        candidate = file_;
        candidate += "." + std::string(tag) + "-" + std::to_string(n) + ".bak";
    }
    fs::copy_file(file_, candidate, fs::copy_options::none, ec);
    if (ec) return makeError(ErrorCode::IoError, "cannot back up to " + candidate.string() + ": " + ec.message());
    return candidate;
}

Result<RobotConfig> FileConfigStore::load() {
    Result<std::string> text = readTextFile(file_);
    if (!text) return text.error();
    Result<Value> parsed = text_.parse(text.value());
    if (!parsed) return makeError(parsed.error().code, file_.string() + ": " + parsed.error().message);
    Result<RobotConfig> config = codec_.fromValue(parsed.value());
    if (!config) return makeError(config.error().code, file_.string() + ": " + config.error().message);
    return config;
}
Status FileConfigStore::save(const RobotConfig& config) {
    return atomicWriteFile(file_, text_.dump(codec_.toValue(config), true));
}

// ---- ProjectConfigRepository ---------------------------------------------------------------------------

FileConfigStore ProjectConfigRepository::projectStore(const fs::path& projectDir) const {
    return FileConfigStore(projectConfigPath(projectDir), "robot setup", text_, codec_);
}

FileConfigStore ProjectConfigRepository::legacyAppDataStore(const fs::path& projectDir) const {
    return FileConfigStore(appDataConfigPath(root_, projectDir), "app data robot setup", text_, codec_);
}

Result<RobotConfig> ProjectConfigRepository::open(const fs::path& projectDir, ConfigSource* usedSource) const {
    FileConfigStore project = projectStore(projectDir);
    if (project.exists()) {
        // An unreadable file is reported as it is. Falling back to an older copy would hide the problem and let
        // the next save overwrite whatever the team put in git.
        Result<RobotConfig> cfg = project.load();
        if (cfg && usedSource != nullptr) *usedSource = ConfigSource::Project;
        return cfg;
    }

    FileConfigStore legacy = legacyAppDataStore(projectDir);
    if (!legacy.exists()) return makeError(ErrorCode::NotFound, "no robot setup at " + project.path().string());
    Result<RobotConfig> cfg = legacy.load();
    if (!cfg) return cfg;
    // Set up with an older version, which kept the setup in app data: move it into the project, once.
    if (Status s = project.save(cfg.value()); !s) {
        // Still usable from app data; the old copy stays until a save into the project succeeds.
        if (logger_ != nullptr)
            logger_->warn("config", "could not move the robot setup into the project, using " + legacy.path().string() +
                                        " for now: " + s.error().message);
    } else {
        // Kept as a backup rather than deleted, but out of the way: a project file deleted later on purpose must
        // not come back from here.
        Result<fs::path> kept = legacy.backup("migrated");
        std::error_code ec;
        if (kept) fs::remove(legacy.path(), ec);
        if (logger_ != nullptr) {
            if (kept && !ec)
                logger_->info("config", "moved the robot setup into " + project.path().string() + "; the old copy is kept as " +
                                            kept.value().string());
            else
                logger_->warn("config", "moved the robot setup into " + project.path().string() + " but could not retire " +
                                            legacy.path().string());
        }
    }
    if (usedSource != nullptr) *usedSource = ConfigSource::MigratedFromAppData;
    return cfg;
}

Status ProjectConfigRepository::save(const fs::path& projectDir, const RobotConfig& config) const {
    FileConfigStore project = projectStore(projectDir);
    // The file is the only copy. One that does not load (hand-edited, a merge conflict, a newer schema) is kept
    // before it is replaced, so finishing setup after a load error never destroys the only copy of someone's work.
    if (project.exists()) {
        if (Result<RobotConfig> current = project.load(); !current && current.error().code != ErrorCode::NotFound) {
            Result<fs::path> kept = project.backup("unreadable");
            if (!kept) return kept.error();
            if (logger_ != nullptr)
                logger_->warn("config", "replaced an unreadable robot setup; the old file is kept as " + kept.value().string());
        }
    }
    return project.save(config);
}

}  // namespace wpiemu::core::config
