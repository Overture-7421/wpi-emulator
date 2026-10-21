#include <algorithm>
#include <iterator>

#include "model/config_ops.h"
#include "wpiemu/sdk/mechanisms/motion_config.h"

namespace wpiemu::app::editor::ops {

using sdk::DeviceKey;
using sdk::MechanismConfig;
using sdk::RobotConfig;
using sdk::SlotAddress;

// ---- device mappings --------------------------------------------------------------------------
std::vector<sdk::SlotMapping>* mappingList(RobotConfig& c, std::string_view owner) {
    if (owner == sdk::slot_owner::kRobot) return &c.robotMappings;
    if (owner == sdk::slot_owner::kDrivetrain) return &c.drivetrain.mappings;
    if (MechanismConfig* m = c.findMechanism(owner)) return &m->mappings;
    if (sdk::SensorConfig* s = findSensor(c, owner)) return &s->mappings;
    return nullptr;
}

const sdk::SlotMapping* findMapping(const RobotConfig& c, const SlotAddress& slot) {
    const auto& list = [&]() -> const std::vector<sdk::SlotMapping>& {
        static const std::vector<sdk::SlotMapping> empty;
        if (slot.owner == sdk::slot_owner::kRobot) return c.robotMappings;
        if (slot.owner == sdk::slot_owner::kDrivetrain) return c.drivetrain.mappings;
        if (const MechanismConfig* m = c.findMechanism(slot.owner)) return m->mappings;
        if (const sdk::SensorConfig* s = c.findSensor(slot.owner)) return s->mappings;
        return empty;
    }();
    for (const auto& m : list)
        if (m.slot == slot) return &m;
    return nullptr;
}

DeviceKey declaredKeyFor(const RobotConfig& c, const DeviceKey& proposed) {
    for (const auto& m : c.allMappings())
        if (m.device.physical() == proposed.physical()) return m.device;
    return proposed;
}

bool setMapping(RobotConfig& c, const SlotAddress& slot, const DeviceKey& device) {
    auto* list = mappingList(c, slot.owner);
    if (list == nullptr) return false;
    for (auto& m : *list)
        if (m.slot == slot) {
            m.device = device;
            return true;
        }
    list->push_back({slot, device});
    return true;
}

bool setMappingReversed(RobotConfig& c, const SlotAddress& slot, bool reversed) {
    auto* list = mappingList(c, slot.owner);
    if (list == nullptr) return false;
    for (auto& m : *list)
        if (m.slot == slot) {
            m.reversed = reversed;
            return true;
        }
    return false;
}

bool clearMapping(RobotConfig& c, const SlotAddress& slot) {
    auto* list = mappingList(c, slot.owner);
    if (list == nullptr) return false;
    const auto before = list->size();
    list->erase(std::remove_if(list->begin(), list->end(), [&](const sdk::SlotMapping& m) { return m.slot == slot; }),
                list->end());
    return list->size() != before;
}

std::vector<SlotAddress> slotsUsing(const RobotConfig& c, const sdk::PhysicalDeviceId& device) {
    std::vector<SlotAddress> out;
    for (const auto& m : c.allMappings())
        if (m.device.physical() == device) out.push_back(m.slot);
    return out;
}

// ---- slots ------------------------------------------------------------------------------------
namespace {
constexpr std::string_view kWheelCountSuffix = ".driveMotorCount";

// Current and maximum motor count behind `countKey`; false when the key is unknown for the owner.
bool motorCountOf(const RobotConfig& c, const EditorCatalog& catalog, std::string_view owner, std::string_view countKey, int& count, int& maxCount) {
    if (owner == sdk::slot_owner::kDrivetrain) {
        if (countKey.size() <= kWheelCountSuffix.size() || countKey.substr(countKey.size() - kWheelCountSuffix.size()) != kWheelCountSuffix) return false;
        const sdk::WheelSpec* w = findWheel(c, countKey.substr(0, countKey.size() - kWheelCountSuffix.size()));
        if (w == nullptr) return false;
        maxCount = sdk::kMaxMotorsPerAxis;
        count = std::clamp(w->driveMotorCount, 1, maxCount);
        return true;
    }
    const MechanismConfig* m = c.findMechanism(owner);
    const auto* info = m != nullptr ? catalog.findTemplate(m->type) : nullptr;
    if (info == nullptr) return false;
    maxCount = 0;
    for (const auto& spec : info->slotSpecs)
        if (spec.countKey == countKey) maxCount = std::max(maxCount, spec.ordinal);
    if (maxCount == 0) return false;
    count = 1;
    if (const sdk::PropertySpec* p = info->properties.find(std::string(countKey))) count = static_cast<int>(p->defaultValue.asInt(1));
    if (const sdk::Value* v = m->properties.find(std::string(countKey))) count = static_cast<int>(v->asInt(count));
    count = std::clamp(count, 1, maxCount);
    return true;
}
}  // namespace

std::string wheelOfSlot(std::string_view slotName) {
    const auto dot = slotName.rfind('.');
    if (dot == std::string_view::npos || dot == 0) return {};
    std::string_view tail = slotName.substr(dot + 1);
    if (tail == sdk::kSteerSlot || tail == sdk::kSteerEncoderSlot) return std::string(slotName.substr(0, dot));
    if (tail.substr(0, 5) != "drive") return {};
    tail.remove_prefix(5);
    for (const char ch : tail)
        if (ch < '0' || ch > '9') return {};
    return std::string(slotName.substr(0, dot));
}

std::vector<MotorGroup> motorGroups(const RobotConfig& c, const EditorCatalog& catalog, std::string_view owner) {
    std::vector<MotorGroup> out;
    for (const auto& spec : slotSpecsFor(c, catalog, owner)) {
        if (spec.type != sdk::SlotType::Motor || spec.countKey.empty()) continue;
        auto it = std::find_if(out.begin(), out.end(), [&](const MotorGroup& g) { return g.countKey == spec.countKey; });
        if (it == out.end()) {
            MotorGroup g;
            g.countKey = spec.countKey;
            if (!motorCountOf(c, catalog, owner, g.countKey, g.count, g.maxCount)) continue;
            out.push_back(std::move(g));
            it = std::prev(out.end());
        }
        it->slotNames.push_back(spec.name);
    }
    return out;
}

void pruneInactiveMappings(RobotConfig& c, const EditorCatalog& catalog, std::string_view owner) {
    auto* list = mappingList(c, owner);
    if (list == nullptr) return;
    std::vector<std::string> active;
    for (const auto& spec : slotSpecsFor(c, catalog, owner)) active.push_back(spec.name);
    list->erase(std::remove_if(list->begin(), list->end(),
                               [&](const sdk::SlotMapping& m) {
                                   if (std::find(active.begin(), active.end(), m.slot.slot) != active.end()) return false;
                                   // Only motor slots that exist for a higher motor number are pruned; unknown slots are left for the validator.
                                   if (owner == sdk::slot_owner::kDrivetrain) {
                                       const std::string wheel = wheelOfSlot(m.slot.slot);
                                       return !wheel.empty() && findWheel(c, wheel) != nullptr &&
                                              m.slot.slot != sdk::steerSlotName(wheel) &&
                                              m.slot.slot != sdk::steerEncoderSlotName(wheel);
                                   }
                                   const MechanismConfig* mech = c.findMechanism(owner);
                                   const auto* info = mech != nullptr ? catalog.findTemplate(mech->type) : nullptr;
                                   if (info == nullptr) return false;
                                   return std::any_of(info->slotSpecs.begin(), info->slotSpecs.end(),
                                                      [&](const sdk::SlotSpec& s) { return s.name == m.slot.slot && !s.countKey.empty(); });
                               }),
                list->end());
}

bool setMotorCount(RobotConfig& c, const EditorCatalog& catalog, std::string_view owner, const std::string& countKey, int count) {
    int current = 0, maxCount = 0;
    if (!motorCountOf(c, catalog, owner, countKey, current, maxCount)) return false;
    count = std::clamp(count, 1, maxCount);
    if (count == current) return false;
    if (owner == sdk::slot_owner::kDrivetrain) {
        findWheel(c, countKey.substr(0, countKey.size() - kWheelCountSuffix.size()))->driveMotorCount = count;
    } else {
        MechanismConfig* m = c.findMechanism(owner);
        if (!m->properties.isObject()) m->properties = sdk::Value::makeObject();
        m->properties[countKey] = count;
    }
    pruneInactiveMappings(c, catalog, owner);
    return true;
}

std::vector<sdk::SlotSpec> slotSpecsFor(const RobotConfig& c, const EditorCatalog& catalog, std::string_view owner) {
    std::vector<sdk::SlotSpec> out;
    if (owner == sdk::slot_owner::kRobot)
        out = sdk::robotSlotSpecs();
    else if (owner == sdk::slot_owner::kDrivetrain)
        out = sdk::drivetrainSlotSpecs(catalog.findDrivetrain(c.drivetrain.spec.kind), c.drivetrain.spec);
    else if (const sdk::SensorConfig* s = c.findSensor(owner))
        out.push_back(sdk::sensorSlotSpec(s->kind));
    else if (const MechanismConfig* m = c.findMechanism(owner))
        if (const auto* info = catalog.findTemplate(m->type)) {
            for (const auto& spec : info->slotSpecs) {
                int count = 0, maxCount = 0;
                if (!spec.countKey.empty() && motorCountOf(c, catalog, owner, spec.countKey, count, maxCount) && spec.ordinal > count) continue;
                out.push_back(spec);
            }
        }
    const auto axes =
        sdk::motionAxisInfos(c, [&](std::string_view type) { return catalog.findTemplate(type); });
    for (auto& slot : out)
        if (sdk::motionFollowerMotor(c, axes, {std::string(owner), slot.name})) {
            slot.required = false;
            slot.description = "Driven by a motion link; no motor mapping is needed.";
        }
    return out;
}

std::vector<std::string> slotOwners(const RobotConfig& c) {
    std::vector<std::string> out{sdk::slot_owner::kRobot, sdk::slot_owner::kDrivetrain};
    for (const auto& m : c.mechanisms) out.push_back(m.id);
    for (const auto& s : c.sensors) out.push_back(s.id);
    return out;
}

std::string slotDisplayName(const RobotConfig& c, const EditorCatalog& catalog, const SlotAddress& slot) {
    std::string ownerName = slot.owner == sdk::slot_owner::kRobot        ? "Robot"
                            : slot.owner == sdk::slot_owner::kDrivetrain ? "Drivetrain"
                                                                         : slot.owner;
    if (const MechanismConfig* m = c.findMechanism(slot.owner)) ownerName = displayNameFor(*m, catalog);
    if (const sdk::SensorConfig* s = c.findSensor(slot.owner)) ownerName = sensorDisplayName(*s);
    std::string slotName = slot.slot;
    for (const auto& s : slotSpecsFor(c, catalog, slot.owner))
        if (s.name == slot.slot) { slotName = s.label.empty() ? s.name : s.label; break; }
    return ownerName + " \xE2\x80\xBA " + slotName;
}

}  // namespace wpiemu::app::editor::ops
