#include "client/robot_file.h"

#include "wpiemu/core/config/config_store.h"
#include "wpiemu/core/match/spec_intake.h"

namespace wpiemu::app::client {
namespace {

std::vector<std::string> splitDetail(const std::string& detail) {
    std::vector<std::string> lines;
    std::size_t start = 0;
    for (;;) {
        const std::size_t at = detail.find("; ", start);
        lines.push_back(detail.substr(start, at == std::string::npos ? std::string::npos : at - start));
        if (at == std::string::npos) return lines;
        start = at + 2;
    }
}

}  // namespace

std::string utf8Of(const std::filesystem::path& path) {
    const std::u8string u8 = path.u8string();
    return std::string(reinterpret_cast<const char*>(u8.data()), u8.size());
}

RobotFileReport checkRobotJson(const std::string& json, const std::string& label, const ClientServices& services) {
    RobotFileReport report;
    report.path = label;
    auto accepted = core::match::acceptRemoteSpec(json, services.pipeline());
    if (!accepted) {
        report.errors = splitDetail(accepted.error().message);
        return report;
    }
    report.clamps = std::move(accepted).value().clamped;

    // The Join carries the file as written: the host (and this client's own intake) clamp it the same way.
    auto parsed = services.json().parse(json);
    auto loaded = parsed ? services.pipeline().load(parsed.value()) : sdk::Result<core::config::LoadedConfig>(parsed.error());
    if (!loaded) {
        report.errors = {loaded.error().message};
        return report;
    }
    report.config = std::move(loaded).value().config;
    report.robotName = report.config->name;
    report.gamePluginId = report.config->gamePluginId;
    report.gameKnown = services.gameOf(report.gamePluginId) != nullptr;
    report.wheelCount = report.config->drivetrain.spec.wheels.size();
    report.zoneCount = report.config->zones.size();
    report.ok = true;
    return report;
}

RobotFileReport loadRobotFile(const std::filesystem::path& file, const ClientServices& services) {
    auto text = core::config::readTextFile(file);
    if (!text) {
        RobotFileReport report;
        report.path = utf8Of(file);
        report.errors = {text.error().message};
        return report;
    }
    return checkRobotJson(text.value(), utf8Of(file), services);
}

}  // namespace wpiemu::app::client
