#pragma once

// The two FRC alliances: one definition for the field configuration, the driver station protocol, game plugins and
// the UI. The driver station's Red1..Blue3 numbering lives with the protocol (ipc/driver_station.h, allianceOf/stationOf).

namespace wpiemu::sdk {

enum class Alliance { Blue, Red };

constexpr Alliance otherAlliance(Alliance a) { return a == Alliance::Blue ? Alliance::Red : Alliance::Blue; }

// Lower-case name used in persisted config, ids and labels ("blue" / "red").
constexpr const char* allianceKey(Alliance a) { return a == Alliance::Blue ? "blue" : "red"; }

}  // namespace wpiemu::sdk
