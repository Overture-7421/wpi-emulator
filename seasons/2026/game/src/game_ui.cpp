#include "wpiemu/plugins/game_2026/game_ui.h"

#include <initializer_list>
#include <string>
#include <utility>

#include "wpiemu/plugins/game_2026/config.h"
#include "wpiemu/plugins/game_2026/game_state.h"

namespace wpiemu::plugins::game_2026 {

namespace {

using sdk::HudAnchor;
using sdk::HudWidgetDescriptor;
using sdk::HudWidgetKind;

HudWidgetDescriptor widget(std::string id, std::string title, HudWidgetKind kind, const char* valueKey, HudAnchor anchor,
                           int order) {
    HudWidgetDescriptor w;
    w.id = "game_2026." + std::move(id);
    w.title = std::move(title);
    w.kind = kind;
    w.valueKey = valueKey;
    w.anchor = anchor;
    w.order = order;
    return w;
}

}  // namespace

std::vector<HudWidgetDescriptor> hudWidgets() {
    std::vector<HudWidgetDescriptor> w;

    HudWidgetDescriptor meter = widget("fuel_meter", "Fuel", HudWidgetKind::Meter, hud::kFuelCount, HudAnchor::BottomCenter, 0);
    meter.maxKey = hud::kFuelCapacity;
    meter.unit = "fuel";
    meter.color = "#ffd000";
    w.push_back(std::move(meter));

    HudWidgetDescriptor board = widget("scoreboard", "Score", HudWidgetKind::Scoreboard, hud::kMatchPhase, HudAnchor::TopCenter, 0);
    board.sides = {{"BLUE", hud::kScoreBlue, hud::kHubBlueStatus, "#1a73e8"}, {"RED", hud::kScoreRed, hud::kHubRedStatus, "#d93025"}};
    w.push_back(std::move(board));
    w.push_back(widget("shots", "Shots fired", HudWidgetKind::Counter, hud::kShots, HudAnchor::BottomLeft, 0));
    w.push_back(widget("fuel_on_field", "Fuel on field", HudWidgetKind::Counter, hud::kFuelOnField, HudAnchor::BottomLeft, 1));
    return w;
}

}  // namespace wpiemu::plugins::game_2026
