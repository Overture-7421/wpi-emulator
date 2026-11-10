#include "wpiemu/core/match/spec_intake.h"

#include <algorithm>

#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/physics/remote_spec_limits.h"

namespace wpiemu::core::match {

namespace {

constexpr std::size_t kMaxReportedIssues = 3;

std::string joinDetail(const std::vector<std::string>& lines) {
    std::string out;
    for (std::size_t i = 0; i < lines.size() && i < kMaxReportedIssues; ++i) {
        if (i != 0) out += "; ";
        out += lines[i];
    }
    if (out.size() > sdk::match::kMaxDetailBytes) out.resize(sdk::match::kMaxDetailBytes);
    return out;
}

bool ignoredForRemote(const sdk::Issue& issue) {
    return issue.code.rfind("slot_", 0) == 0 || issue.code.rfind("device_", 0) == 0;
}

std::string describe(const sdk::SpecViolation& v) { return v.path + ": " + v.message; }

}  // namespace

sdk::Result<AcceptedSpec> acceptRemoteSpec(std::string_view json, const config::ConfigPipeline& pipeline) {
    using sdk::ErrorCode;
    if (json.empty()) return sdk::makeError(ErrorCode::InvalidArgument, "the robot description is empty");
    if (json.size() > sdk::match::kMaxRobotSpecBytes)
        return sdk::makeError(ErrorCode::InvalidArgument, "the robot description is larger than " +
                                                              std::to_string(sdk::match::kMaxRobotSpecBytes) + " bytes");

    const config::JsonValueCodec codec;
    sdk::Result<sdk::Value> parsed = codec.parse(json);
    if (!parsed) return sdk::makeError(ErrorCode::InvalidArgument, "robot description is not valid JSON: " + parsed.error().message);
    sdk::Result<config::LoadedConfig> loaded = pipeline.load(parsed.value());
    if (!loaded) return sdk::makeError(ErrorCode::InvalidArgument, "robot description rejected: " + loaded.error().message);

    std::vector<std::string> errors;
    for (const sdk::Issue& issue : loaded.value().issues)
        if (issue.severity == sdk::Severity::Error && !ignoredForRemote(issue))
            errors.push_back(issue.path.empty() ? issue.message : issue.path + ": " + issue.message);
    if (!errors.empty()) return sdk::makeError(ErrorCode::InvalidArgument, joinDetail(errors));

    AcceptedSpec out;
    out.config = std::move(loaded).value().config;
    const std::vector<sdk::SpecViolation> violations = sdk::checkRemoteSpec(out.config.chassis, out.config.drivetrain.spec);
    std::vector<std::string> fatal;
    for (const sdk::SpecViolation& v : violations) {
        if (v.code == sdk::remote_spec_code::kTooMany || v.code == sdk::remote_spec_code::kTooFew) fatal.push_back(describe(v));
        else out.clamped.push_back(describe(v));
    }
    if (!fatal.empty()) return sdk::makeError(ErrorCode::InvalidArgument, joinDetail(fatal));

    if (!violations.empty()) {
        sdk::RemoteRobotSpec clamped = sdk::clampRemoteSpec(out.config.chassis, out.config.drivetrain.spec);
        out.config.chassis = std::move(clamped.chassis);
        out.config.drivetrain.spec = std::move(clamped.drivetrain);
        const std::vector<sdk::SpecViolation> left = sdk::checkRemoteSpec(out.config.chassis, out.config.drivetrain.spec);
        if (!left.empty()) {
            std::vector<std::string> lines;
            for (const sdk::SpecViolation& v : left) lines.push_back(describe(v));
            return sdk::makeError(ErrorCode::InvalidArgument, joinDetail(lines));
        }
    }
    return out;
}

}  // namespace wpiemu::core::match
