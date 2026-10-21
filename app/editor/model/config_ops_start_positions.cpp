#include <algorithm>
#include <cctype>

#include "model/config_ops.h"
#include "model/config_ops_internal.h"
#include "wpiemu/sdk/core/strings.h"

namespace wpiemu::app::editor::ops {

using sdk::RobotConfig;

// ---- start positions --------------------------------------------------------------------------
std::string uniqueStartPositionId(const RobotConfig& c, std::string base) {
    return detail::uniqueSuffixed(std::move(base), "start", "_",
                                  [&](const std::string& id) { return c.findStartPosition(id) != nullptr; });
}

std::string startPositionDisplayName(const sdk::StartPosition& p) { return p.label.empty() ? p.id : p.label; }

std::string addStartPosition(RobotConfig& c) {
    sdk::StartPosition p;
    p.id = uniqueStartPositionId(c, "start");
    p.label = "Start " + std::to_string(c.startPositions.size() + 1);
    p.pose = sdk::RobotStartPose{2.f, 4.f, 0.f};
    std::string id = p.id;
    c.startPositions.push_back(std::move(p));
    return id;
}

bool removeStartPosition(RobotConfig& c, std::string_view id) { return sdk::detail::eraseById(c.startPositions, id); }

std::string duplicateStartPosition(RobotConfig& c, std::string_view id) {
    const sdk::StartPosition* original = c.findStartPosition(id);
    if (original == nullptr) return {};
    sdk::StartPosition copy = *original;
    copy.id = uniqueStartPositionId(c, copy.id);
    copy.label = startPositionDisplayName(*original) + " (copy)";
    std::string newId = copy.id;
    c.startPositions.push_back(std::move(copy));
    return newId;
}

namespace {

// "Blue left" -> "Red left", "BLUE" -> "RED", "blue" -> "red"; nothing to swap: nullopt.
std::optional<std::string> swapAllianceWord(const std::string& label, sdk::Alliance from, sdk::Alliance to) {
    const std::string needle(sdk::allianceKey(from));
    const std::size_t at = sdk::asciiLower(label).find(needle);
    if (at == std::string::npos) return std::nullopt;
    std::string word(sdk::allianceKey(to));
    const std::string found = label.substr(at, needle.size());
    if (std::all_of(found.begin(), found.end(), [](unsigned char ch) { return std::isupper(ch) != 0; })) {
        std::transform(word.begin(), word.end(), word.begin(), [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
    } else if (std::isupper(static_cast<unsigned char>(found.front())) != 0) {
        word[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(word[0])));
    }
    std::string out = label;
    out.replace(at, needle.size(), word);
    return out;
}

}  // namespace

std::string copyStartPositionToOtherAlliance(RobotConfig& c, std::string_view id, const sdk::FieldFrame& frame) {
    const sdk::StartPosition* original = c.findStartPosition(id);
    if (original == nullptr) return {};
    const auto flipped = sdk::flipToOtherAlliance(original->pose, frame);
    if (!flipped) return {};
    sdk::StartPosition copy;
    copy.alliance = sdk::otherAlliance(original->alliance);
    copy.pose = *flipped;
    const std::string base = startPositionDisplayName(*original);
    copy.label = swapAllianceWord(base, original->alliance, copy.alliance)
                     .value_or(base + (copy.alliance == sdk::Alliance::Red ? " (Red)" : " (Blue)"));
    copy.id = uniqueStartPositionId(c, original->id + (copy.alliance == sdk::Alliance::Red ? "_red" : "_blue"));
    std::string newId = copy.id;
    c.startPositions.push_back(std::move(copy));
    return newId;
}

sdk::StartPosition* findStartPosition(RobotConfig& c, std::string_view id) { return c.findStartPosition(id); }

}  // namespace wpiemu::app::editor::ops
