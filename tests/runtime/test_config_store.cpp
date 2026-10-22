#include <doctest/doctest.h>

#include "runtime_fixtures.h"
#include "wpiemu/core/config/config_store.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/sdk/testing/environment.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core::config;
using wpiemu::sdk::testing::envOf;
using wpiemu::test_runtime::sampleConfig;
using wpiemu::test_runtime::ScratchDir;
namespace fs = std::filesystem;

namespace {

struct Codecs {
    JsonValueCodec json;
    RobotConfigCodec codec;
};

}  // namespace

TEST_CASE("app data root per OS") {
    CHECK(appDataRoot(Platform::Windows, envOf({{"APPDATA", "C:\\Users\\a\\AppData\\Roaming"}})).value() ==
          fs::path("C:\\Users\\a\\AppData\\Roaming") / "WpiEmulator");
    CHECK(appDataRoot(Platform::Windows, envOf({{"USERPROFILE", "C:\\Users\\a"}})).value() ==
          fs::path("C:\\Users\\a") / "AppData" / "Roaming" / "WpiEmulator");
    CHECK(appDataRoot(Platform::MacOS, envOf({{"HOME", "/Users/a"}})).value() ==
          fs::path("/Users/a") / "Library" / "Application Support" / "WpiEmulator");
    CHECK(appDataRoot(Platform::Linux, envOf({{"XDG_DATA_HOME", "/x/share"}, {"HOME", "/home/a"}})).value() ==
          fs::path("/x/share") / "wpi-emulator");
    CHECK(appDataRoot(Platform::Linux, envOf({{"HOME", "/home/a"}})).value() == fs::path("/home/a") / ".local" / "share" / "wpi-emulator");
    CHECK(appDataRoot(Platform::Linux, envOf({})).error().code == ErrorCode::NotFound);
    CHECK(appDataRoot(Platform::Windows, nullptr).error().code == ErrorCode::NotFound);
    CHECK(appDataRoot(currentPlatform(), &readEnvironment).ok());  // the real environment of the test machine has a home
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

TEST_CASE("repository: the project file is the only copy") {
    ScratchDir dir;
    Codecs c;
    const fs::path appData = dir.path() / "appdata";
    const fs::path projectA = dir.path() / "robotA";
    const fs::path projectB = dir.path() / "robotB";
    ProjectConfigRepository repo(appData, c.json, c.codec);

    CHECK(repo.open(projectA).error().code == ErrorCode::NotFound);

    RobotConfig a = sampleConfig();
    a.name = "A";
    RobotConfig b = sampleConfig();
    b.name = "B";
    REQUIRE(repo.save(projectA, a).ok());
    REQUIRE(repo.save(projectB, b).ok());
    CHECK(fs::exists(projectA / ".wpiemu" / "robot.json"));
    CHECK_FALSE(fs::exists(appData));  // nothing is written outside the project

    ConfigSource source = ConfigSource::MigratedFromAppData;
    CHECK(repo.open(projectA, &source).value().name == "A");
    CHECK(source == ConfigSource::Project);
    CHECK(repo.open(projectB).value().name == "B");

    // A pull or a teammate's change replaces the file: the next open reads it, nothing in between.
    RobotConfig pulled = sampleConfig();
    pulled.name = "Pulled";
    REQUIRE(repo.projectStore(projectA).save(pulled).ok());
    CHECK(repo.open(projectA).value().name == "Pulled");

    // A clone on another machine (another app-data root) reads the same file.
    ProjectConfigRepository other(dir.path() / "appdata2", c.json, c.codec);
    CHECK(other.open(projectA).value().name == "Pulled");
}

TEST_CASE("repository: an unreadable project file is reported and kept until replaced") {
    ScratchDir dir;
    Codecs c;
    const fs::path project = dir.path() / "robot";
    ProjectConfigRepository repo(dir.path() / "appdata", c.json, c.codec);
    const fs::path file = projectConfigPath(project);
    const auto backups = [&] {
        int n = 0;
        for (const auto& e : fs::directory_iterator(file.parent_path())) n += e.path().extension() == ".bak" ? 1 : 0;
        return n;
    };

    // An older version's setup in app data must not hide a broken project file.
    RobotConfig legacy = sampleConfig();
    legacy.name = "Legacy";
    REQUIRE(repo.legacyAppDataStore(project).save(legacy).ok());
    dir.write("robot/.wpiemu/robot.json", "<<<<<<< HEAD merge conflict");
    const auto opened = repo.open(project);
    REQUIRE_FALSE(opened.ok());
    CHECK(opened.error().code != ErrorCode::NotFound);
    CHECK(ScratchDir::read(file) == "<<<<<<< HEAD merge conflict");

    // Saving over it (finishing setup after the error) keeps the broken file first.
    RobotConfig fresh = sampleConfig();
    fresh.name = "Fresh";
    REQUIRE(repo.save(project, fresh).ok());
    CHECK(repo.open(project).value().name == "Fresh");
    CHECK(backups() == 1);
    CHECK(ScratchDir::read(fs::path(file.string() + ".unreadable.bak")) == "<<<<<<< HEAD merge conflict");

    // Later saves over a good file make no more backups.
    fresh.name = "Fresh again";
    REQUIRE(repo.save(project, fresh).ok());
    CHECK(backups() == 1);
}

TEST_CASE("repository: an unreadable file is not replaced when it cannot be kept") {
    ScratchDir dir;
    Codecs c;
    const fs::path project = dir.path() / "robot";
    ProjectConfigRepository repo(dir.path() / "appdata", c.json, c.codec);
    const fs::path file = projectConfigPath(project);
    dir.write("robot/.wpiemu/robot.json", "{ hand edited");
    for (int n = 1; n < 1000; ++n) {
        fs::path occupied = file;
        occupied += n == 1 ? ".unreadable.bak" : ".unreadable-" + std::to_string(n) + ".bak";
        REQUIRE(fs::create_directory(occupied));
    }

    const Status result = repo.save(project, sampleConfig());
    REQUIRE_FALSE(result.ok());
    CHECK(result.error().code == ErrorCode::IoError);
    CHECK(result.error().message.find("cannot back up") != std::string::npos);
    CHECK(ScratchDir::read(file) == "{ hand edited");
}

TEST_CASE("repository: an older version's app-data setup moves into the project once") {
    ScratchDir dir;
    Codecs c;
    const fs::path project = dir.path() / "robot";
    ProjectConfigRepository repo(dir.path() / "appdata", c.json, c.codec);
    const fs::path legacyPath = repo.legacyAppDataStore(project).path();
    RobotConfig legacy = sampleConfig();
    legacy.name = "Set up before the project file";
    REQUIRE(repo.legacyAppDataStore(project).save(legacy).ok());

    ConfigSource source = ConfigSource::Project;
    CHECK(repo.open(project, &source).value().name == "Set up before the project file");
    CHECK(source == ConfigSource::MigratedFromAppData);
    CHECK(fs::exists(projectConfigPath(project)));
    CHECK_FALSE(fs::exists(legacyPath));                                     // retired...
    CHECK(fs::exists(fs::path(legacyPath.string() + ".migrated.bak")));      // ...but kept

    CHECK(repo.open(project, &source).value().name == "Set up before the project file");
    CHECK(source == ConfigSource::Project);

    // Deleting the project file on purpose resets the project; the old copy does not come back.
    fs::remove(projectConfigPath(project));
    CHECK(repo.open(project).error().code == ErrorCode::NotFound);
}

TEST_CASE("repository: an older setup stays usable when the project cannot take it") {
    ScratchDir dir;
    Codecs c;
    const fs::path project = dir.path() / "robot";
    ProjectConfigRepository repo(dir.path() / "appdata", c.json, c.codec);
    RobotConfig legacy = sampleConfig();
    legacy.name = "Legacy";
    REQUIRE(repo.legacyAppDataStore(project).save(legacy).ok());
    dir.write("robot/.wpiemu", "a file where the folder should be");

    ConfigSource source = ConfigSource::Project;
    CHECK(repo.open(project, &source).value().name == "Legacy");
    CHECK(source == ConfigSource::MigratedFromAppData);
    CHECK(repo.legacyAppDataStore(project).exists());  // not retired: the move did not happen
    CHECK(repo.save(project, legacy).error().code == ErrorCode::IoError);
}
