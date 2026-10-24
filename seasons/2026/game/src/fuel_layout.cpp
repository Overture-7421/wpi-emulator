#include "wpiemu/plugins/game_2026/fuel_layout.h"

#include <algorithm>
#include <cmath>

#include "wpiemu/plugins/game_2026/field_dimensions.h"

namespace wpiemu::plugins::game_2026 {

namespace {

// Grid cell centres filling `count` cells row by row; the last (partial) row is centred.
void appendBlock(std::vector<sdk::Vec3>& out, int count, int columns, const sdk::Vec3& center, float spacingX,
                 float spacingY, float xSign) {
    if (count <= 0) return;
    const int rows = (count + columns - 1) / columns;
    int placed = 0;
    for (int r = 0; r < rows; ++r) {
        const int inRow = std::min(columns, count - placed);
        for (int c = 0; c < inRow; ++c) {
            const float x = (static_cast<float>(c) - static_cast<float>(inRow - 1) * 0.5f) * spacingX;
            const float y = (static_cast<float>(r) - static_cast<float>(rows - 1) * 0.5f) * spacingY;
            out.push_back({center.x + xSign * x, center.y + y, dim::kFuelRadiusM});
        }
        placed += inRow;
    }
}

}  // namespace

std::vector<sdk::Vec3> initialFuelPositions(const GameConfig& config, const FieldLayout& layout) {
    std::vector<sdk::Vec3> out;
    out.reserve(static_cast<std::size_t>(std::max(config.neutralFuel, 0) + 2 * std::max(config.depotFuel, 0)));
    appendBlock(out, config.neutralFuel, dim::kNeutralColumns, {dim::kFieldLengthM / 2, dim::kFieldWidthM / 2, 0.f},
                dim::kNeutralSpacingXM, dim::kNeutralSpacingYM, 1.f);
    for (const Alliance a : kAlliances) {
        const DepotSpec& depot = layout.depot(a);
        // Fill the depot from the alliance wall outward: columns run along X, rows along Y.
        appendBlock(out, std::min(config.depotFuel, dim::kDepotColumns * dim::kDepotRows), dim::kDepotColumns,
                    depot.center, dim::kNeutralSpacingXM, dim::kNeutralSpacingYM, depot.xDir);
    }
    return out;
}

}  // namespace wpiemu::plugins::game_2026
