#include "wpiemu/core/runtime/sensor_system.h"

#include <algorithm>
#include <cmath>

namespace wpiemu::core::runtime {

using namespace wpiemu::sdk;

namespace {

double number(const Value& props, const char* key, double fallback) {
    const Value* v = props.find(key);
    return v != nullptr && v->isNumber() ? v->asDouble() : fallback;
}

bool isLinear(const Value& props) {
    const Value* mode = props.find(sensor_prop::kMode);
    return mode != nullptr && mode->isString() && mode->asString() == sensor_prop::kModeLinear;
}

}  // namespace

double idleDistanceM(const Value& props) {
    if (isLinear(props)) return number(props, sensor_prop::kDistanceAtMinM, 1.0);
    return number(props, sensor_prop::kFarM, 1.0);
}

double distanceReadingM(const Value& props, double input) {
    if (isLinear(props)) {
        const double lo = number(props, sensor_prop::kInputMin, 0.0);
        const double hi = number(props, sensor_prop::kInputMax, 1.0);
        const double d0 = number(props, sensor_prop::kDistanceAtMinM, 1.0);
        const double d1 = number(props, sensor_prop::kDistanceAtMaxM, 0.1);
        if (hi == lo) return d0;
        const double t = std::clamp((input - lo) / (hi - lo), 0.0, 1.0);
        return d0 + t * (d1 - d0);
    }
    return input >= number(props, sensor_prop::kThreshold, 1.0) ? number(props, sensor_prop::kNearM, 0.1)
                                                                : number(props, sensor_prop::kFarM, 1.0);
}

bool idleBeamBreakReading(const Value& props) {
    const Value* broken = props.find(sensor_prop::kBrokenValue);
    return !(broken == nullptr || broken->asBool(true));
}

bool beamBreakReading(const Value& props, double input) {
    const Value* brokenValue = props.find(sensor_prop::kBrokenValue);
    const bool whenBroken = brokenValue == nullptr || brokenValue->asBool(true);
    const bool broken = input >= number(props, sensor_prop::kThreshold, 1.0);
    return broken ? whenBroken : !whenBroken;
}

SensorSystem::SensorSystem(const RobotConfig& config) : config_(config) {
    states_.reserve(config.sensors.size());
    for (const SensorConfig& s : config.sensors) {
        const double idle = s.kind == SensorKind::Distance ? idleDistanceM(s.properties)
                                                           : (idleBeamBreakReading(s.properties) ? 1.0 : 0.0);
        states_.push_back({s.id, std::nullopt, idle});
    }
}

void SensorSystem::publish(SnapshotBuilder& builder) const {
    for (const SensorState& s : states_) builder.addSensor(s);
}

void SensorSystem::step(const IGameState* state, ISensorWriter& hub) {
    unavailable_.clear();
    states_.clear();
    for (const SensorConfig& s : config_.sensors) {
        std::optional<double> input;
        if (state != nullptr && !s.source.empty()) input = state->gameValue(s.source);
        if (!input || !std::isfinite(*input)) {
            input.reset();
            if (!s.source.empty()) unavailable_.push_back(s.id);
        }
        const SlotAddress slot{s.id, kSensorSlot};
        double reading;
        if (s.kind == SensorKind::Distance) {
            reading = input ? distanceReadingM(s.properties, *input) : idleDistanceM(s.properties);
            hub.writeDistance(slot, reading);
        } else {
            const bool level = input ? beamBreakReading(s.properties, *input) : idleBeamBreakReading(s.properties);
            reading = level ? 1.0 : 0.0;
            hub.writeDigital(slot, level);
        }
        states_.push_back({s.id, input, reading});
    }
}

std::vector<HubWarning> SensorSystem::warnings() const {
    std::vector<HubWarning> out;
    for (const std::string& id : unavailable_)
        out.push_back({Severity::Warning, DeviceId{}, "sensor_source_unavailable",
                       "sensor '" + id + "' reads idle: its game value is not published by the game state"});
    return out;
}

}  // namespace wpiemu::core::runtime
