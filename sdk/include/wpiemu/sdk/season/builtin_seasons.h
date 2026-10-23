#pragma once

#include "wpiemu/sdk/core/result.h"

namespace wpiemu::sdk {

class SeasonRegistry;

// Aggregation seam for season profiles. DECLARED here so app code can call it without any manifest knowledge; DEFINED by
// the `wpiemu_seasons_builtin` target, whose source CMake generates from seasons/*/season.json. Returns the first
// registration error (all seasons are still attempted).
Status registerBuiltinSeasons(SeasonRegistry& registry);

}  // namespace wpiemu::sdk
