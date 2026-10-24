#include <doctest/doctest.h>

#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/season/builtin_seasons.h"
#include "wpiemu/sdk/season/season_registry.h"

using namespace wpiemu::sdk;

namespace {

SeasonProfile profile(int year, bool isDefault = false) {
    SeasonProfile p;
    p.year = year;
    p.isDefault = isDefault;
    p.wpilibVersion = std::to_string(year) + ".1.0";
    p.verifiedWpilibVersions = {p.wpilibVersion};
    p.installFolder = std::to_string(year);
    p.halAbi = "abi_x";
    p.launchProfile = "gradlerio-modern";
    return p;
}

}  // namespace

TEST_CASE("season registry: add, find, list in year order and the default season") {
    SeasonRegistry registry;
    CHECK(registry.defaultSeason() == nullptr);
    REQUIRE(registry.add(profile(3002)).ok());
    REQUIRE(registry.add(profile(3001, true)).ok());
    REQUIRE(registry.size() == 2);
    CHECK(registry.list().front().year == 3001);
    REQUIRE(registry.find(3002) != nullptr);
    CHECK(registry.find(3003) == nullptr);
    REQUIRE(registry.defaultSeason() != nullptr);
    CHECK(registry.defaultSeason()->year == 3001);
}

TEST_CASE("season registry: invalid, duplicate and second-default profiles are rejected") {
    SeasonRegistry registry;
    REQUIRE(registry.add(profile(3001, true)).ok());
    CHECK_FALSE(registry.add(profile(0)).ok());
    CHECK_FALSE(registry.add(profile(3001)).ok());
    CHECK_FALSE(registry.add(profile(3002, true)).ok());
    for (auto mutate : {+[](SeasonProfile& p) { p.wpilibVersion.clear(); }, +[](SeasonProfile& p) { p.halAbi.clear(); },
                        +[](SeasonProfile& p) { p.launchProfile.clear(); }}) {
        SeasonProfile broken = profile(3005);
        mutate(broken);
        CHECK_FALSE(registry.add(broken).ok());
    }
    CHECK(registry.size() == 1);
}

TEST_CASE("season profile: versions match exactly, by trailing wildcard, and ignore a leading v") {
    CHECK(versionMatches("2026.3.4", "v2026.3.4"));
    CHECK(versionMatches("v2026.3.4", "2026.3.4"));
    CHECK(versionMatches("2026.*", "2026.7"));
    CHECK_FALSE(versionMatches("2026.3.4", "2026.3.40"));
    CHECK_FALSE(versionMatches("2026.*", "2027.1"));
    SeasonProfile p = profile(3001);
    p.verifiedWpilibVersions = {"3001.1.0", "3001.2.*"};
    CHECK(p.verifiesWpilib("3001.1.0"));
    CHECK(p.verifiesWpilib("3001.2.7"));
    CHECK_FALSE(p.verifiesWpilib("3001.1.1"));
}

TEST_CASE("builtin seasons: manifests register one default and every named game plugin agrees on its year") {
    SeasonRegistry seasons;
    REQUIRE(registerBuiltinSeasons(seasons).ok());
    REQUIRE(seasons.size() >= 1);
    CHECK(seasons.defaultSeason() != nullptr);

    GamePluginRegistry plugins;
    REQUIRE(registerBuiltinGamePlugins(plugins).ok());
    for (const SeasonProfile& season : seasons.list()) {
        CAPTURE(season.year);
        CHECK(season.verifiesWpilib(season.wpilibVersion));
        const IGameModule* module = plugins.findByYear(season.year);
        if (season.gamePluginId.empty()) {
            CHECK(module == nullptr);
        } else {
            REQUIRE(module != nullptr);
            CHECK(module->metadata().id == season.gamePluginId);
        }
    }
    for (const IGameModule* module : plugins.list()) {
        const SeasonProfile* season = seasons.find(module->metadata().year);
        REQUIRE(season != nullptr);
        CHECK(season->gamePluginId == module->metadata().id);
    }
}
