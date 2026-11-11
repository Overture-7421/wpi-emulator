#include "wpiemu/core/match/hud_keys.h"

#include <algorithm>

namespace wpiemu::core::match {

HudKeyTable HudKeyTable::fromKeys(std::vector<std::string> keys) {
    std::erase_if(keys, [](const std::string& k) { return k.empty(); });
    std::sort(keys.begin(), keys.end());
    keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
    if (keys.size() > kMaxKeys) keys.resize(kMaxKeys);
    HudKeyTable table;
    table.keys_ = std::move(keys);
    return table;
}

HudKeyTable HudKeyTable::fromDescriptors(std::span<const sdk::HudWidgetDescriptor> descriptors,
                                         std::span<const std::string> extraKeys) {
    std::vector<std::string> keys(extraKeys.begin(), extraKeys.end());
    for (const sdk::HudWidgetDescriptor& d : descriptors) {
        keys.push_back(d.valueKey);
        keys.push_back(d.maxKey);
        for (const sdk::HudScoreSide& side : d.sides) {
            keys.push_back(side.scoreKey);
            keys.push_back(side.statusKey);
        }
    }
    return fromKeys(std::move(keys));
}

std::optional<std::uint16_t> HudKeyTable::idOf(std::string_view key) const {
    const auto it = std::lower_bound(keys_.begin(), keys_.end(), key,
                                     [](const std::string& a, std::string_view b) { return std::string_view(a) < b; });
    if (it == keys_.end() || *it != key) return std::nullopt;
    return static_cast<std::uint16_t>(it - keys_.begin() + 1);
}

std::optional<std::string_view> HudKeyTable::keyOf(std::uint16_t id) const {
    if (id == 0 || id > keys_.size()) return std::nullopt;
    return std::string_view(keys_[static_cast<std::size_t>(id) - 1]);
}

}  // namespace wpiemu::core::match
