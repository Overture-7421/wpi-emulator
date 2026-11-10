#pragma once

// The player's robot.json: read from a file anywhere (the client has no project), checked with the host's own intake and
// reported in terms a person can act on. The same decode, defaults, validation and fairness clamps run here and on the host
// (core/match acceptRemoteSpec), so what passes here is what the host accepts, and `clamps` lists the fields it will correct.

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "client/client_services.h"
#include "wpiemu/sdk/config/robot_config.h"

namespace wpiemu::app::client {

struct RobotFileReport {
    std::string path;
    bool ok = false;
    std::optional<sdk::RobotConfig> config;   // as written in the file (decoded and defaulted); what the Join carries
    std::string robotName;
    std::string gamePluginId;                 // what the file selects; empty: none
    bool gameKnown = false;                   // this build has that game plugin
    std::vector<std::string> errors;          // why the file cannot be used (one line each); empty when ok
    std::vector<std::string> clamps;          // fields the host will correct to plausible FRC ranges, one line each
    std::size_t wheelCount = 0;
    std::size_t zoneCount = 0;
};

RobotFileReport loadRobotFile(const std::filesystem::path& file, const ClientServices& services);
// A path as UTF-8 text (what a report and the settings store).
std::string utf8Of(const std::filesystem::path& path);
// The same for text already in memory (`label` names it in the report).
RobotFileReport checkRobotJson(const std::string& json, const std::string& label, const ClientServices& services);

}  // namespace wpiemu::app::client
