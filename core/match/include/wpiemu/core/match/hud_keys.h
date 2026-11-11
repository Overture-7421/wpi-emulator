#pragma once

// The HUD key table of a practice match: the ONE place that maps a game plugin's string HUD keys ("game_2026.fuel.count") to
// the u16 `HudValue::keyId` the snapshots carry. The wire has no field for the table, so both sides derive it from what
// they share: the plugin they run (Join::gamePluginId). The table is the sorted, de-duplicated set of every key the
// plugin's HUD descriptors reference (valueKey, maxKey and each scoreboard side's scoreKey and statusKey) plus the
// keys the caller adds; the id of a key is its 1-based position in that order, so the host and a client of the same plugin
// build the same table. Id 0 is never assigned. A HUD value whose key is not in the table is not sent.

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/game/ui_descriptors.h"

namespace wpiemu::core::match {

class HudKeyTable {
public:
    static constexpr std::size_t kMaxKeys = 0xFFFF;

    HudKeyTable() = default;
    static HudKeyTable fromKeys(std::vector<std::string> keys);
    static HudKeyTable fromDescriptors(std::span<const sdk::HudWidgetDescriptor> descriptors,
                                       std::span<const std::string> extraKeys = {});

    std::optional<std::uint16_t> idOf(std::string_view key) const;
    std::optional<std::string_view> keyOf(std::uint16_t id) const;
    std::size_t size() const { return keys_.size(); }
    // Sorted keys; keys()[i] has id i + 1.
    const std::vector<std::string>& keys() const { return keys_; }

private:
    std::vector<std::string> keys_;
};

}  // namespace wpiemu::core::match
