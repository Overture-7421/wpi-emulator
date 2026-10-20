#pragma once

// What the user has selected in the project tree / viewport / validation list. A plain value type so
// every view (tree, viewport, inspector, validation panel) can share it and tests can build it.

#include <string>
#include <string_view>

namespace wpiemu::app::editor {

enum class SelKind {
    None,
    Robot,       // the whole robot (name, battery)
    Chassis,     // hull box
    ChassisCom,  // centre-of-mass marker
    Drivetrain,  // drivetrain kind / tire settings
    Wheel,       // owner unused, sub = wheel name
    Primitive,   // owner = "chassis" or a mechanism id, index = primitive index
    Mechanism,   // owner = mechanism id
    Slot,        // owner = slot owner (mechanism id, "drivetrain", "robot"), sub = slot name
    Overlay,     // plugin-declared volume: owner = mechanism id or "" (plugin level), sub = property key
    Plugin,      // the selected game plugin's config section
    Zone,        // owner = zone id (RobotConfig::zones)
    Camera,      // owner = camera id (RobotConfig::cameras)
    Sensor,      // owner = sensor id (RobotConfig::sensors)
    Device,       // a detected device: owner = serialized DeviceKey (see device_mime.h)
};

struct SelectionRef {
    SelKind kind = SelKind::None;
    std::string owner;
    std::string sub;
    int index = -1;

    bool operator==(const SelectionRef&) const = default;
    bool isNone() const { return kind == SelKind::None; }

    static SelectionRef robot() { return {SelKind::Robot, {}, {}, -1}; }
    static SelectionRef chassis() { return {SelKind::Chassis, {}, {}, -1}; }
    static SelectionRef com() { return {SelKind::ChassisCom, {}, {}, -1}; }
    static SelectionRef drivetrain() { return {SelKind::Drivetrain, {}, {}, -1}; }
    static SelectionRef wheel(std::string name) { return {SelKind::Wheel, {}, std::move(name), -1}; }
    static SelectionRef primitive(std::string owner, int index) { return {SelKind::Primitive, std::move(owner), {}, index}; }
    static SelectionRef mechanism(std::string id) { return {SelKind::Mechanism, std::move(id), {}, -1}; }
    static SelectionRef slot(std::string owner, std::string slot) { return {SelKind::Slot, std::move(owner), std::move(slot), -1}; }
    static SelectionRef overlay(std::string owner, std::string key) { return {SelKind::Overlay, std::move(owner), std::move(key), -1}; }
    static SelectionRef plugin() { return {SelKind::Plugin, {}, {}, -1}; }
    static SelectionRef zone(std::string id) { return {SelKind::Zone, std::move(id), {}, -1}; }
    static SelectionRef camera(std::string id) { return {SelKind::Camera, std::move(id), {}, -1}; }
    static SelectionRef sensor(std::string id) { return {SelKind::Sensor, std::move(id), {}, -1}; }
    static SelectionRef device(std::string key) { return {SelKind::Device, std::move(key), {}, -1}; }

    // Stable textual key (tree expansion state, QML item ids, tests).
    std::string key() const {
        return std::to_string(static_cast<int>(kind)) + "|" + owner + "|" + sub + "|" + std::to_string(index);
    }
};

}  // namespace wpiemu::app::editor
