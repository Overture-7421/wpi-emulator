#include <doctest/doctest.h>

#include "launcher_test_support.h"
#include "wpiemu/core/launcher/extension_locator.h"
#include "wpiemu/core/launcher/project_inspector.h"
#include "wpiemu/sdk/season/builtin_seasons.h"
#include "wpiemu/sdk/season/season_registry.h"
#include "wpiemu/sdk/testing/environment.h"

// The extensibility contract of docs/reference/seasons.md: a new season on an existing HAL ABI and launch profile is data.
// Nothing below names a real season; every fact is read from the profile, as the build reads it from a season pack.

using namespace wpiemu::sdk;
using namespace wpiemu::core::launcher;
using wpiemu::sdk::testing::envOf;
using wpiemu::test_runtime::ScratchDir;
using wpiemu::test_runtime::makeRequest;
namespace fs = std::filesystem;

namespace {

SeasonProfile syntheticSeason() {
    SeasonProfile p;
    p.year = 2999;
    p.channel = SeasonChannel::Preview;
    p.wpilibVersion = "2999.0.0-beta-1";
    p.verifiedWpilibVersions = {"2999.0.0-beta-*"};
    p.installFolder = "2999_beta1";
    p.halAbi = "systemcore";
    p.launchProfile = std::string(kLaunchModern);
    p.javaMajor = 25;
    return p;
}

}  // namespace

TEST_CASE("season extensibility: a new season registers beside the built-ins and the default is untouched") {
    SeasonRegistry seasons;
    REQUIRE(registerBuiltinSeasons(seasons).ok());
    const int defaultYear = seasons.defaultSeason()->year;
    const std::size_t before = seasons.size();
    REQUIRE(seasons.add(syntheticSeason()).ok());
    CHECK(seasons.size() == before + 1);
    CHECK(seasons.defaultSeason()->year == defaultYear);
    REQUIRE(seasons.find(2999) != nullptr);
    CHECK(seasons.find(2999)->gamePluginId.empty());   // platform-only: runs core features
}

TEST_CASE("season extensibility: its launch profile, install folder, extension and version policy all come from the profile") {
    const SeasonProfile season = syntheticSeason();

    // Launch planning: the profile decides the Gradle tasks and the descriptor, not the year.
    ScratchDir dir;
    LaunchRequest request = makeRequest(dir, LaunchMode::ExternalDirect);
    request.wpilibYear = season.year;
    request.launchProfile = season.launchProfile;
    CHECK(gradleTaskName(request) == std::string(findLaunchProfile(season.launchProfile)->javaTask));
    CHECK(externalTaskName(request) == std::string(findLaunchProfile(season.launchProfile)->javaExternalTask));
    const auto describe = planExternalDescribe(request, false);
    REQUIRE(describe.ok());
    CHECK(describe.value().args.back() == externalTaskName(request));

    // WPILib discovery: the folder the manifest names, or a newer suffixed one the manifest does not know yet.
    ScratchDir roots;
    roots.write("wpilib/2999_beta1/jdk/bin/java", "x");
    CHECK(findWpilibInstall(season.year, {roots.path() / "wpilib"}, season.installFolder).has_value());
    roots.write("wpilib/2999_beta2/jdk/bin/java", "x");
    CHECK(findWpilibInstall(season.year, {roots.path() / "wpilib"})->home == roots.path() / "wpilib" / "2999_beta2");

    // Extension lookup: the year-qualified directory.
    const auto candidates = halsimExtensionCandidates("/opt/app/bin", envOf({}), season.year);
    REQUIRE_FALSE(candidates.empty());
    CHECK(candidates.front().generic_string().find("/2999/") != std::string::npos);

    // Version policy: a new revision of the season is unverified until the manifest lists it; it warns, never refuses.
    CHECK(season.verifiesWpilib("2999.0.0-beta-2"));
    CHECK_FALSE(season.verifiesWpilib("2999.1.0"));
}
