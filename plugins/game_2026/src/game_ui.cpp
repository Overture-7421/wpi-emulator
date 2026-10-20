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

    HudWidgetDescriptor red = widget("score_red", "Red fuel points", HudWidgetKind::Counter, hud::kScoreRed, HudAnchor::TopLeft, 0);
    red.color = "#d93025";
    w.push_back(std::move(red));
    HudWidgetDescriptor blue = widget("score_blue", "Blue fuel points", HudWidgetKind::Counter, hud::kScoreBlue, HudAnchor::TopRight, 0);
    blue.color = "#1a73e8";
    w.push_back(std::move(blue));
    HudWidgetDescriptor redHub = widget("hub_red", "Red hub", HudWidgetKind::Badge, hud::kHubRedStatus, HudAnchor::TopLeft, 1);
    redHub.color = "#d93025";
    w.push_back(std::move(redHub));
    HudWidgetDescriptor blueHub = widget("hub_blue", "Blue hub", HudWidgetKind::Badge, hud::kHubBlueStatus, HudAnchor::TopRight, 1);
    blueHub.color = "#1a73e8";
    w.push_back(std::move(blueHub));

    w.push_back(widget("shots", "Shots fired", HudWidgetKind::Counter, hud::kShots, HudAnchor::BottomLeft, 0));
    w.push_back(widget("fuel_on_field", "Fuel on field", HudWidgetKind::Counter, hud::kFuelOnField, HudAnchor::BottomLeft, 1));
    return w;
}

}  // namespace wpiemu::plugins::game_2026
