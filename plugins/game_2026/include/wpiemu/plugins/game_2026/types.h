#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace wpiemu::plugins::game_2026 {

enum class Alliance : std::uint8_t { Red = 0, Blue = 1 };

inline constexpr std::array<Alliance, 2> kAlliances{Alliance::Red, Alliance::Blue};

constexpr std::size_t index(Alliance a) { return static_cast<std::size_t>(a); }
constexpr Alliance opponent(Alliance a) { return a == Alliance::Red ? Alliance::Blue : Alliance::Red; }
constexpr const char* allianceName(Alliance a) { return a == Alliance::Red ? "red" : "blue"; }

// What a physics user tag refers to (ShapeDesc::userTag is returned in sensor events).
enum class EntityKind : std::uint8_t { None = 0, Fuel = 1, HubSensor = 2, CorralSensor = 3 };

constexpr std::uint64_t makeTag(EntityKind kind, Alliance alliance, std::uint32_t id = 0) {
    return (static_cast<std::uint64_t>(kind) << 56) | (static_cast<std::uint64_t>(alliance) << 48) | id;
}
constexpr EntityKind tagKind(std::uint64_t tag) { return static_cast<EntityKind>(tag >> 56); }
constexpr Alliance tagAlliance(std::uint64_t tag) { return static_cast<Alliance>((tag >> 48) & 0xFFu); }
constexpr std::uint32_t tagId(std::uint64_t tag) { return static_cast<std::uint32_t>(tag & 0xFFFFFFFFu); }

}  // namespace wpiemu::plugins::game_2026
