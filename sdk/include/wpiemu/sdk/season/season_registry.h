#pragma once

#include <algorithm>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/season/season_profile.h"

namespace wpiemu::sdk {

// Owns the registered season profiles. An INSTANCE created by the composition root (app/tests); no global state.
// Built-ins arrive through registerBuiltinSeasons() (builtin_seasons.h), generated from seasons/*/season.json.
class SeasonRegistry {
public:
    // Validates the profile (year > 0, unique year, ABI/launch profile/version present, one default).
    Status add(SeasonProfile profile) {
        if (profile.year <= 0) return makeError(ErrorCode::InvalidArgument, "season year must be > 0");
        const std::string where = "season " + std::to_string(profile.year);
        if (profile.wpilibVersion.empty()) return makeError(ErrorCode::InvalidArgument, where + " has no WPILib version");
        if (profile.halAbi.empty()) return makeError(ErrorCode::InvalidArgument, where + " names no HAL ABI");
        if (profile.launchProfile.empty()) return makeError(ErrorCode::InvalidArgument, where + " names no launch profile");
        if (find(profile.year) != nullptr) return makeError(ErrorCode::AlreadyExists, "duplicate " + where);
        if (profile.isDefault && defaultSeason() != nullptr)
            return makeError(ErrorCode::AlreadyExists, where + " is a second default season");
        seasons_.push_back(std::move(profile));
        std::sort(seasons_.begin(), seasons_.end(), [](const SeasonProfile& a, const SeasonProfile& b) { return a.year < b.year; });
        return okStatus();
    }

    const SeasonProfile* find(int year) const {
        for (const SeasonProfile& s : seasons_)
            if (s.year == year) return &s;
        return nullptr;
    }
    // The season a project with an unknown year falls back to; nullptr until one is registered as default.
    const SeasonProfile* defaultSeason() const {
        for (const SeasonProfile& s : seasons_)
            if (s.isDefault) return &s;
        return nullptr;
    }
    // The season whose game plugin has this id, or nullptr.
    const SeasonProfile* findByGamePlugin(const std::string& pluginId) const {
        for (const SeasonProfile& s : seasons_)
            if (!pluginId.empty() && s.gamePluginId == pluginId) return &s;
        return nullptr;
    }
    // Ascending year order.
    const std::vector<SeasonProfile>& list() const { return seasons_; }
    std::size_t size() const { return seasons_.size(); }

private:
    std::vector<SeasonProfile> seasons_;
};

}  // namespace wpiemu::sdk
