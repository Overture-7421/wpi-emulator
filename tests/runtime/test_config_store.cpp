#include <doctest/doctest.h>

#include <map>

#include "runtime_fixtures.h"
#include "wpiemu/core/config/config_store.h"
#include "wpiemu/core/config/json_codec.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core::config;
using wpiemu::test_runtime::sampleConfig;
using wpiemu::test_runtime::ScratchDir;
namespace fs = std::filesystem;

namespace {
EnvLookup fakeEnv(std::map<std::string, std::string> vars) {
    return [vars = std::move(vars)](std::string_view name) -> std::optional<std::string> {
        const auto it = vars.find(std::string(name));
        if (it == vars.end()) return std::nullopt;
        return it->second;
    };
}

struct Codecs {
    JsonValueCodec json;
    RobotConfigCodec codec;
};

}  // namespace

TEST_CASE("app data root per OS") {
    CHECK(appDataRoot(Platform::Windows, fakeEnv({{"APPDATA", "C:\\Users\\a\\AppData\\Roaming"}})).value() ==
          fs::path("C:\\Users\\a\\AppData\\Roaming") / "WpiEmulator");
    CHECK(appDataRoot(Platform::Windows, fakeEnv({{"USERPROFILE", "C:\\Users\\a"}})).value() ==
          fs::path("C:\\Users\\a") / "AppData" / "Roaming" / "WpiEmulator");
    CHECK(appDataRoot(Platform::MacOS, fakeEnv({{"HOME", "/Users/a"}})).value() ==
          fs::path("/Users/a") / "Library" / "Application Support" / "WpiEmulator");
    CHECK(appDataRoot(Platform::Linux, fakeEnv({{"XDG_DATA_HOME", "/x/share"}, {"HOME", "/home/a"}})).value() ==
          fs::path("/x/share") / "wpi-emulator");
    CHECK(appDataRoot(Platform::Linux, fakeEnv({{"HOME", "/home/a"}})).value() == fs::path("/home/a") / ".local" / "share" / "wpi-emulator");
    CHECK(appDataRoot(Platform::Linux, fakeEnv({})).error().code == ErrorCode::NotFound);
    CHECK(appDataRoot(Platform::Windows, nullptr).error().code == ErrorCode::NotFound);
    CHECK(appDataRoot(currentPlatform(), systemEnv()).ok());  // the real environment of the test machine has a home
}

TEST_CASE("project key: stable, path-hashed, readable") {
    const std::string a = projectKeyForPath("C:/Users/a/robot", true);
    CHECK(a == projectKeyForPath("C:/Users/a/robot/", true));       // trailing slash
    CHECK(a == projectKeyForPath("c:\\users\\A\\ROBOT", true));      // case + separators on case-insensitive FS
    CHECK(a != projectKeyForPath("C:/Users/b/robot", true));         // same folder name, different path
    CHECK(projectKeyForPath("/home/A/robot", false) != projectKeyForPath("/home/a/robot", false));
    CHECK(a.rfind("robot-", 0) == 0);
    CHECK(a.size() == std::string("robot-").size() + 16);
    CHECK(projectKeyForPath("/x/we ird:name*", false).find(' ') == std::string::npos);
    CHECK(projectKeyForPath("/", false).rfind("project-", 0) == 0);
    CHECK(projectKeyForPath("/home/a/" + std::string(100, 'x'), false).size() <= 32 + 1 + 16);

    ScratchDir dir;
    CHECK(projectKey(dir.path()) == projectKey(dir.path() / "." / ""));  // canonicalised
}

TEST_CASE("atomic write: creates directories, replaces content, leaves no temp files") {
    ScratchDir dir;
    const fs::path target = dir.path() / "a" / "b" / "file.txt";
    REQUIRE(atomicWriteFile(target, "first").ok());
    CHECK(ScratchDir::read(target) == "first");
    REQUIRE(atomicWriteFile(target, "second, longer content").ok());
    CHECK(ScratchDir::read(target) == "second, longer content");
    REQUIRE(atomicWriteFile(target, "").ok());
    CHECK(ScratchDir::read(target).empty());
    int files = 0;
    for (const auto& e : fs::directory_iterator(target.parent_path())) {
        ++files;
        CHECK(e.path().extension() != ".tmp");
    }
    CHECK(files == 1);
    // the destination directory cannot be created below a regular file
    CHECK_FALSE(atomicWriteFile(target / "x" / "y.txt", "z").ok());
    CHECK(readTextFile(dir.path() / "missing").error().code == ErrorCode::NotFound);
}

TEST_CASE("atomic write: replacement failure retains its cause after successful cleanup") {
    ScratchDir dir;
    const fs::path target = dir.path() / "target";
    fs::create_directory(target);
    dir.write((fs::path("target") / "keep.txt").string(), "untouched");
    const fs::path probe = dir.write("probe", "probe");
    std::error_code replaceError;
    fs::rename(probe, target, replaceError);
    REQUIRE(replaceError);
    REQUIRE(fs::remove(probe));

    const Status result = atomicWriteFile(target, "new content");
    REQUIRE_FALSE(result.ok());
    CHECK(result.error().code == ErrorCode::IoError);
    CHECK(result.error().message.find("cannot replace") != std::string::npos);
    CHECK(result.error().message.find(replaceError.message()) != std::string::npos);
    CHECK(ScratchDir::read(target / "keep.txt") == "untouched");
    for (const auto& entry : fs::directory_iterator(dir.path())) CHECK(entry.path() == target);
}

TEST_CASE("file store: save/load round trip, NotFound, parse errors name the file") {
    ScratchDir dir;
    Codecs c;
    FileConfigStore store(dir.path() / "robot.json", "test", c.json, c.codec);
    CHECK(store.load().error().code == ErrorCode::NotFound);
    CHECK(store.describe().find("test") != std::string::npos);

    REQUIRE(store.save(sampleConfig()).ok());
    const auto loaded = store.load();
    REQUIRE(loaded.ok());
    CHECK(loaded.value().name == "Sample");

    dir.write("broken.json", "{ not json");
    FileConfigStore broken(dir.path() / "broken.json", "test", c.json, c.codec);
    const auto r = broken.load();
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == ErrorCode::ParseError);
    CHECK(r.error().message.find("broken.json") != std::string::npos);

    dir.write("newer.json", "{\"schemaVersion\": 99}");
    CHECK(FileConfigStore(dir.path() / "newer.json", "t", c.json, c.codec).load().error().code == ErrorCode::VersionMismatch);
}

TEST_CASE("file store: a file with another schema version is refused and left untouched") {
    ScratchDir dir;
    Codecs c;
    const std::string original = "{\n  \"schemaVersion\": 0,\n  \"name\": \"Old\"\n}\n";
    const fs::path file = dir.write("robot.json", original);
    FileConfigStore store(file, "test", c.json, c.codec);

    const auto loaded = store.load();
    REQUIRE_FALSE(loaded.ok());
    CHECK(loaded.error().code == ErrorCode::VersionMismatch);
    CHECK(ScratchDir::read(file) == original);   // no upgrade write-back
    int files = 0;
    for ([[maybe_unused]] const auto& e : fs::directory_iterator(dir.path())) ++files;
    CHECK(files == 1);                           // and no backup copy
}

TEST_CASE("file store: backups never overwrite each other") {
    ScratchDir dir;
    Codecs c;
    FileConfigStore store(dir.path() / "robot.json", "test", c.json, c.codec);
    REQUIRE(store.save(sampleConfig()).ok());
    const auto first = store.backup("v0");
    REQUIRE(first.ok());
    CHECK(first.value().filename() == "robot.json.v0.bak");
    const auto second = store.backup("v0");
    REQUIRE(second.ok());
    CHECK(second.value().filename() == "robot.json.v0-2.bak");
}
TEST_CASE("file store: backup of a missing file fails cleanly, unwritable save reports an error") {
    ScratchDir dir;
    Codecs c;
    FileConfigStore store(dir.path() / "none.json", "t", c.json, c.codec);
    CHECK(store.backup("v1").error().code == ErrorCode::NotFound);
    dir.write("blocker", "x");
    FileConfigStore blocked(dir.path() / "blocker" / "robot.json", "t", c.json, c.codec);
    CHECK(blocked.save(sampleConfig()).error().code == ErrorCode::IoError);
}

TEST_CASE("repository: app-data copy, project export/import, per-project keys") {
    ScratchDir dir;
    Codecs c;
    const fs::path appData = dir.path() / "appdata";
    const fs::path projectA = dir.path() / "robotA";
    const fs::path projectB = dir.path() / "robotB";
    fs::create_directories(projectA);
    fs::create_directories(projectB);
    ProjectConfigRepository repo(appData, c.json, c.codec);

    CHECK(repo.open(projectA).error().code == ErrorCode::NotFound);

    RobotConfig a = sampleConfig();
    a.name = "A";
    RobotConfig b = sampleConfig();
    b.name = "B";
    REQUIRE(repo.save(projectA, a).ok());
    REQUIRE(repo.save(projectB, b).ok());
    CHECK_FALSE(fs::exists(projectConfigPath(projectA)));  // not exported unless asked
    CHECK(repo.appDataStore(projectA).path() != repo.appDataStore(projectB).path());

    ConfigSource source = ConfigSource::Project;
    CHECK(repo.open(projectA, &source).value().name == "A");
    CHECK(source == ConfigSource::AppData);
    CHECK(repo.open(projectB).value().name == "B");

    REQUIRE(repo.exportToProject(projectA).ok());
    CHECK(fs::exists(projectA / ".wpiemu" / "robot.json"));
    REQUIRE(repo.save(projectB, b, true).ok());
    CHECK(fs::exists(projectB / ".wpiemu" / "robot.json"));

    // a fresh machine (empty app data) opens the shared project file and caches it
    ProjectConfigRepository other(dir.path() / "appdata2", c.json, c.codec);
    const auto imported = other.open(projectA, &source);
    REQUIRE(imported.ok());
    CHECK(source == ConfigSource::Project);
    CHECK(imported.value().name == "A");
    CHECK(other.appDataStore(projectA).exists());

    // importFromProject overwrites the working copy (backing it up first)
    a.name = "A-edited-locally";
    REQUIRE(repo.save(projectA, a).ok());
    const auto reimported = repo.importFromProject(projectA);
    REQUIRE(reimported.ok());
    CHECK(reimported.value().name == "A");
    CHECK(repo.open(projectA).value().name == "A");
    bool hasBackup = false;
    for (const auto& e : fs::directory_iterator(repo.appDataStore(projectA).path().parent_path())) hasBackup |= e.path().extension() == ".bak";
    CHECK(hasBackup);

    CHECK(repo.importFromProject(dir.path() / "nothing").error().code == ErrorCode::NotFound);
    CHECK(repo.exportToProject(dir.path() / "nothing").error().code == ErrorCode::NotFound);
}

TEST_CASE("repository: import preserves local bytes when every backup candidate is occupied") {
    ScratchDir dir;
    Codecs c;
    const fs::path project = dir.path() / "robot";
    ProjectConfigRepository repo(dir.path() / "appdata", c.json, c.codec);
    RobotConfig cfg = sampleConfig();
    cfg.name = "Shared";
    REQUIRE(repo.save(project, cfg, true).ok());
    cfg.name = "Local edits";
    REQUIRE(repo.save(project, cfg).ok());
    const fs::path local = repo.appDataStore(project).path();
    const std::string original = ScratchDir::read(local);
    for (int n = 1; n < 1000; ++n) {
        fs::path occupied = local;
        occupied += n == 1 ? ".pre-import.bak" : ".pre-import-" + std::to_string(n) + ".bak";
        REQUIRE(fs::create_directory(occupied));
    }

    const auto result = repo.importFromProject(project);
    REQUIRE_FALSE(result.ok());
    CHECK(result.error().code == ErrorCode::IoError);
    CHECK(result.error().message.find("cannot back up") != std::string::npos);
    CHECK(ScratchDir::read(local) == original);
    CHECK(repo.open(project).value().name == "Local edits");
    // The destination itself is still replaceable: only the protective backup failed.
    cfg.name = "Explicit save";
    REQUIRE(repo.save(project, cfg).ok());
    CHECK(repo.open(project).value().name == "Explicit save");
}
