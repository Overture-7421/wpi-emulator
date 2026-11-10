#pragma once

// Turns the robot description of a Join into the spec the session accepts, trusting nothing the client sent:
//   1. size cap (kMaxRobotSpecBytes), JSON parse (bounded depth);
//   2. the REAL config pipeline (decode, defaults, validation): there is no second spec format. Validation errors about
//      mappings of slots and devices (issue codes starting "slot_" or "device_") are ignored: a remote robot has no
//      WPILib program, so nothing is mapped;
//   3. refuse a wheel count outside the limits (spec.too_many, spec.too_few): the setpoints the client sends follow its own
//      wheel list, so the list is accepted as sent or not at all;
//   4. clamp everything else to the plausible-FRC ranges (sdk::clampRemoteSpec) and report each clamped field;
//   5. move each zone's own pose into the zone envelope of the clamped chassis (sdk::clampRemoteZonePose), the same bound a
//      client's commanded zone poses get, so the rest pose a robot's zones have is always inside it.
// Pure apart from the pipeline's plugin registration (sim thread only, like the rest of the host).

#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/core/config/config_pipeline.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/result.h"

namespace wpiemu::core::match {

struct AcceptedSpec {
    sdk::RobotConfig config;              // decoded, defaulted, validated and clamped
    std::vector<std::string> clamped;     // one line per field that was out of range and was corrected
};

// On failure the error message is the Join rejection detail (<= kMaxDetailBytes): the first validation messages.
sdk::Result<AcceptedSpec> acceptRemoteSpec(std::string_view json, const config::ConfigPipeline& pipeline);

}  // namespace wpiemu::core::match
