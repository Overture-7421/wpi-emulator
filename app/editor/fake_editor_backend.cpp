#include "fake_editor_backend.h"

#include <algorithm>

#include "wpiemu/sdk/config/property_builders.h"
#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/devices/device_identity.h"
#include "wpiemu/sdk/dynamics/motor_model.h"
#include "wpiemu/sdk/mechanisms/attachment.h"

namespace wpiemu::app::editor {

using sdk::Capability;
using sdk::CapabilitySet;
using sdk::PropertySpec;
using sdk::PropertyType;
using sdk::PropertyUnit;

namespace {

using sdk::props::box;
using sdk::props::choice;
using sdk::props::integer;
using sdk::props::real;

PropertySpec intCount(const char* key, const char* label, const char* group, int def, int max) {
    PropertySpec p = integer(key, label, group, def, 1, max);
    p.step = 1.0;
    return p;
}

sdk::SlotSpec slot(const char* name, const char* label, sdk::SlotType type, bool required, const char* description = "") {
    sdk::SlotSpec s;
    s.name = name;
    s.label = label;
    s.type = type;
    s.required = required;
    s.description = description;
    return s;
}

// Motor `ordinal` (1-based) of a gearbox whose motor count is the property `motorCount`.
sdk::SlotSpec motorSlot(const char* name, const char* label, int ordinal, bool required) {
    sdk::SlotSpec s = slot(name, label, sdk::SlotType::Motor, required);
    s.countKey = "motorCount";
    s.ordinal = ordinal;
    return s;
}

sdk::PrimitiveSpec prim(const char* name, sdk::PrimitiveKind kind, sdk::Vec3 pos, sdk::Vec3 size, const char* color,
                        float radius = 0.05f, float height = 0.1f, bool lieOnY = false) {
    sdk::PrimitiveSpec p;
    p.name = name;
    p.kind = kind;
    p.local.position = pos;
    if (lieOnY) p.local.rotation = sdk::Quat::fromAxisAngle({1.f, 0.f, 0.f}, static_cast<float>(sdk::kPi * 0.5));
    p.sizeM = size;
    p.radiusM = radius;
    p.heightM = height;
    p.color = color;
    return p;
}

std::vector<sdk::EnumOption> motorChoices() {
    std::vector<sdk::EnumOption> options;
    for (const sdk::MotorPreset& m : sdk::motors::presets()) options.push_back({m.spec.name, m.label});
    return options;
}

sdk::MechanismTemplateInfo templ(const char* type, const char* name, const char* category, const char* description,
                                 std::vector<sdk::SlotSpec> slots, std::vector<PropertySpec> props,
                                 std::vector<sdk::PrimitiveSpec> geometry, std::vector<sdk::LinkSpec> links = {},
                                 const char* origin = "core") {
    sdk::MechanismTemplateInfo t;
    t.type = type;
    t.displayName = name;
    t.category = category;
    t.description = description;
    t.origin = origin;
    t.slotSpecs = std::move(slots);
    t.properties.properties = std::move(props);
    for (auto& p : t.properties.properties)
        if (p.group == "Motor") {
            p.section = sdk::PropertySection::Drive;
            p.axis = "axis";
        }
    t.defaultGeometry = std::move(geometry);
    t.links = std::move(links);
    for (const auto& s : t.slotSpecs)
        if (s.type == sdk::SlotType::Motor && s.ordinal > 1) {
            t.motionAxes.back().motorSlots.push_back(s.name);
        } else if (s.type == sdk::SlotType::Motor) {
            const bool deploy = s.name == "deploy";
            t.motionAxes.push_back(
                {deploy ? "deploy" : "axis",
                 sdk::PlantKind::Angular,
                 {s.name},
                 std::string(type) == "mechanism" ? "motion" : "",
                 deploy ? "hasDeploy" : ""});
        }
    return t;
}

sdk::LinkSpec link(const char* name, const char* label, sdk::Vec3 pos) {
    sdk::LinkSpec l;
    l.name = name;
    l.label = label;
    l.local.position = pos;
    return l;
}

const CapabilitySet kTalon = Capability::MotorOutput | Capability::RotorFeedback | Capability::PositionSensor |
                             Capability::VelocitySensor | Capability::LimitInputs | Capability::SupplyVoltage;

}  // namespace

sdk::DiscoveredDevice FakeEditorBackend::makeDevice(const std::string& family, const std::string& type, int id, const std::string& bus,
                                                    CapabilitySet caps, const std::string& vendor) {
    sdk::DiscoveredDevice d;
    // Stable runtime id derived from the address: the fake has no global counter.
    const std::size_t h = std::hash<std::string>{}(family + '/' + type) ^ (static_cast<std::size_t>(id + 1) * 2654435761u);
    d.id = sdk::DeviceId{static_cast<std::uint32_t>(h % 0xFFFFFFFEu) + 1u};
    d.descriptor.family = family;
    d.descriptor.type = type;
    d.descriptor.vendor = vendor;
    d.descriptor.id = id;
    d.descriptor.bus = bus;
    d.descriptor.capabilities = caps;
    d.live = true;
    return d;
}

EditorCatalog FakeEditorBackend::sampleCatalog() {
    EditorCatalog cat;
    const char* kMech = "Mechanism";
    const char* kMotor = "Motor";

    // The one core template: a single motor-driven degree of freedom (Motion = rotary | arm | linear).
    cat.mechanismTemplates.push_back(templ(
        "mechanism", "Mechanism", "Mechanism", "One motor-driven degree of freedom: rotary, arm or linear.",
        {motorSlot("motor", "Motor", 1, true), motorSlot("motor2", "Motor 2", 2, false), motorSlot("motor3", "Motor 3", 3, false),
         slot("encoder", "Position sensor", sdk::SlotType::PositionSensor, false),
         slot("lowerLimit", "Reverse limit switch", sdk::SlotType::LimitSwitch, false),
         slot("upperLimit", "Forward limit switch", sdk::SlotType::LimitSwitch, false)},
        {choice("motion", "Type", kMech, "rotary", {{"rotary", "Rotary"}, {"arm", "Arm"}, {"linear", "Linear"}}),
         intCount("motorCount", "Motor count", kMotor, 1, 3),
         choice("motorType", "Motor model", kMotor, "Kraken X60", motorChoices()),
         real("gearRatio", "Gear ratio", kMotor, 10.0, 0.1, 500.0, PropertyUnit::Ratio, {}, 0.5),
         real("inertiaKgM2", "Moment of inertia", kMech, 0.01, 0.0001, 10.0, PropertyUnit::KgM2, {}, 0.001),
         real("massKg", "Mass", kMech, 2.0, 0.01, 100.0, PropertyUnit::Kilograms, {}, 0.1),
         real("minPosition", "Minimum position", kMech, 0.0, -10.0, 10.0, PropertyUnit::None, {}, 0.01),
         real("maxPosition", "Maximum position", kMech, 0.0, -10.0, 10.0, PropertyUnit::None, {}, 0.01)},
        {prim("mover", sdk::PrimitiveKind::Box, {0.10f, 0.f, 0.f}, {0.20f, 0.05f, 0.05f}, "#9b59b6")},
        {link("mover", "Moving part", {0.f, 0.f, 0.05f})}));
    // ---- sample mechanisms with a plugin-scope volume (a mechanism property of type Box3) ----
    sdk::Box3 zone;
    zone.pose.position = {0.3f, 0.f, 0.f};
    zone.sizeM = {0.4f, 0.6f, 0.25f};
    cat.mechanismTemplates.push_back(templ(
        "sample.intake", "Roller intake", "Intake", "Roller with a deploy arm and a reach volume.",
        {slot("motor", "Roller motor", sdk::SlotType::Motor, true), slot("deploy", "Deploy motor", sdk::SlotType::Motor, false),
         slot("deployEncoder", "Deploy encoder", sdk::SlotType::PositionSensor, false)},
        {box("pickupZone", "Pickup zone", "Reach", zone, "Where the roller reaches (a Box3 property: gizmo-editable)."),
         integer("capacity", "Capacity", "Reach", 30, 1, 100),
         real("gearRatio", "Gear ratio", kMotor, 3.0, 0.5, 60.0, PropertyUnit::Ratio, {}, 0.1),
         choice("motorType", "Motor", kMotor, "NEO", motorChoices()),
         real("activeThresholdRpm", "Active above", "Reach", 20.0, 0.0, 600.0, PropertyUnit::RadiansPerSecond, {}, 1.0)},
        {prim("roller", sdk::PrimitiveKind::Cylinder, {0.f, 0.f, 0.f}, {}, "#ffb347", 0.03f, 0.55f, true),
         prim("frame", sdk::PrimitiveKind::Box, {-0.06f, 0.f, 0.02f}, {0.1f, 0.55f, 0.05f}, "#9aa5b1")}));

    cat.mechanismTemplates.push_back(templ(
        "sample.shooter", "Flywheel shooter", "Shooter", "Flywheel with a feeder and an exit link.",
        {slot("flywheel", "Flywheel motor", sdk::SlotType::Motor, true), slot("feeder", "Feeder motor", sdk::SlotType::Motor, true),
         slot("encoder", "Flywheel encoder", sdk::SlotType::VelocitySensor, false)},
        {real("wheelDiameterM", "Wheel diameter", "Flywheel", 0.1016, 0.03, 0.3, PropertyUnit::Meters, {}, 0.005),
         real("efficiency", "Launch efficiency", "Flywheel", 0.8, 0.1, 1.0, PropertyUnit::Ratio, {}, 0.01),
         real("fireRateHz", "Fire rate", "Feeding", 8.0, 0.5, 30.0, PropertyUnit::Hertz, {}, 0.5),
         real("feedThresholdRpm", "Feeding above", "Feeding", 30.0, 0.0, 600.0, PropertyUnit::RadiansPerSecond, {}, 1.0),
         choice("motorType", "Motor", kMotor, "Kraken X60", motorChoices())},
        {prim("flywheel", sdk::PrimitiveKind::Cylinder, {0.f, 0.f, 0.f}, {}, "#d1495b", 0.05f, 0.12f, true),
         prim("housing", sdk::PrimitiveKind::Box, {-0.06f, 0.f, 0.f}, {0.14f, 0.2f, 0.12f}, "#9aa5b1")},
        {link("exit", "Exit point", {0.1f, 0.f, 0.f})}));

    cat.drivetrains = {{"swerve", {"Swerve", "Independently steered and driven modules."}},
                       {"tank", {"Tank", "Left / right skid steer."}},
                       {"mecanum", {"Mecanum", "Four mecanum wheels."}}};

    // ---- plugin config ----
    cat.pluginId = "game_2026";
    cat.pluginDisplayName = "REBUILT (2026)";
    cat.pluginSchema.schemaVersion = 1;
    cat.pluginSchema.properties.properties = {
        integer("preloadFuel", "Preloaded fuel", "Robot", 8, 0, 100, "Fuel the robot starts a match with."),
        real("fuelDiameterM", "Fuel diameter", "Fuel", 0.15, 0.05, 0.4, PropertyUnit::Meters, {}, 0.005)};

    // ---- zone kinds the plugin owns ----
    sdk::ZoneKindInfo pickup;
    pickup.kind = "game_2026.pickup";
    pickup.displayName = "Pickup";
    pickup.description = "Absorbs field fuel inside its volume while it is active.";
    pickup.shape = sdk::ZoneShape::Volume;
    pickup.properties.properties = {integer("capacity", "Capacity (fuel)", "Pickup", 30, 1, 100),
                                    real("absorbRateHz", "Absorb rate", "Pickup", 10.0, 0.5, 60.0, PropertyUnit::Hertz, {}, 0.5)};
    pickup.defaultSizeM = {0.4f, 0.6f, 0.25f};
    pickup.defaultLocal.position = {0.3f, 0.f, 0.f};
    pickup.color = "#ff9944";
    cat.zoneKinds.push_back(pickup);

    sdk::ZoneKindInfo shooter;
    shooter.kind = "game_2026.shooter";
    shooter.displayName = "Shooter";
    shooter.description = "Launches fuel from its muzzle along +X while it is active.";
    shooter.shape = sdk::ZoneShape::Muzzle;
    shooter.properties.properties = {real("fireRateHz", "Fire rate", "Shooter", 8.0, 0.5, 30.0, PropertyUnit::Hertz, {}, 0.5),
                                     real("exitSpeedMps", "Exit speed", "Shooter", 10.0, 1.0, 30.0, PropertyUnit::MetersPerSecond, {}, 0.5),
                                     real("spreadRad", "Spread", "Shooter", 0.05, 0.0, 0.5, PropertyUnit::Radians, {}, 0.01)};
    shooter.defaultLocal.position = {0.1f, 0.f, 0.f};
    shooter.color = "#d1495b";
    cat.zoneKinds.push_back(shooter);

    cat.gameValues.push_back({"game_2026.fuel.count", "Fuel in robot", "Fuel currently held by the robot.", PropertyUnit::Count});
    cat.gameValues.push_back({"game_2026.fuel.capacity", "Fuel capacity", "How much fuel the robot can hold.", PropertyUnit::Count});
    cat.gameValues.push_back({"game_2026.fuel.fraction", "Fuel fill", "Held fuel divided by capacity (0 to 1).", PropertyUnit::Ratio});
    cat.fieldFrame = {16.541f, 8.069f, sdk::FieldSymmetry::Rotational};
    sdk::Box3 hopper;
    hopper.pose.position = {-0.05f, 0.f, 0.f};
    hopper.sizeM = {0.4f, 0.5f, 0.3f};

    sdk::EditorPanelDescriptor hopperPanel;
    hopperPanel.id = "game_2026.hopper";
    hopperPanel.title = "Fuel hopper";
    hopperPanel.scope = sdk::PanelScope::PluginConfig;
    hopperPanel.properties.properties = {integer("capacity", "Fuel capacity", "Hopper", 40, 1, 200, "The fuel meter shows this as 100%."),
                                         box("hopperVolume", "Hopper volume", "Hopper", hopper, "Where stored fuel is drawn (chassis frame).")};
    hopperPanel.overlays.push_back({"hopperVolume", "Hopper", "#44cc88", true});
    hopperPanel.order = 1;
    cat.pluginPanels.push_back(hopperPanel);

    // A mechanism-scope panel: only mechanisms of the listed templates get it.
    sdk::EditorPanelDescriptor aiming;
    aiming.id = "game_2026.aiming";
    aiming.title = "Aiming";
    aiming.scope = sdk::PanelScope::Mechanism;
    aiming.mechanismTypes = {"sample.shooter"};
    aiming.properties.properties = {real("hoodAngleRad", "Hood angle", "Aiming", 0.7, 0.0, 1.5, PropertyUnit::Radians, {}, 0.01)};
    aiming.order = 2;
    cat.pluginPanels.push_back(aiming);
    return cat;
}

sdk::RobotConfig FakeEditorBackend::sampleConfig() {
    sdk::RobotConfig c;
    c.name = "Sample Robot";
    c.project.language = "java";
    c.project.wpilibYear = 2026;
    c.gamePluginId = "game_2026";
    c.chassis.sizeM = {0.76f, 0.76f, 0.1f};
    c.chassis.massKg = 52.f;
    c.chassis.comHeightM = 0.22f;
    c.chassis.groundClearanceM = 0.03f;
    c.chassis.primitives.push_back(prim("bumper_front", sdk::PrimitiveKind::Box, {0.40f, 0.f, -0.1f}, {0.06f, 0.88f, 0.12f}, "#c0392b"));
    c.chassis.primitives.back().collides = true;
    c.chassis.primitives.push_back(prim("bumper_rear", sdk::PrimitiveKind::Box, {-0.40f, 0.f, -0.1f}, {0.06f, 0.88f, 0.12f}, "#c0392b"));

    c.drivetrain.spec.kind = "swerve";
    const struct { const char* name; float x; float y; } modules[] = {{"fl", 0.28f, 0.28f}, {"fr", 0.28f, -0.28f}, {"bl", -0.28f, 0.28f}, {"br", -0.28f, -0.28f}};
    int canId = 1;
    for (const auto& m : modules) {
        sdk::WheelSpec w;
        w.name = m.name;
        w.kind = sdk::WheelKind::Swerve;
        w.steered = true;
        w.positionM = {m.x, m.y, w.radiusM};
        c.drivetrain.spec.wheels.push_back(w);
        for (const char* what : {"drive", "steer"}) {
            sdk::DeviceKey key{"phoenix6", "talonfx", canId++, "canivore", -1};
            c.drivetrain.mappings.push_back({{sdk::slot_owner::kDrivetrain, std::string(m.name) + "." + what}, key});
        }
    }
    c.robotMappings.push_back({{sdk::slot_owner::kRobot, "imu"}, {"phoenix6", "pigeon2", 13, "canivore", -1}});

    const EditorCatalog cat = sampleCatalog();
    auto add = [&](const char* type, const char* id, const char* label, sdk::Vec3 mount, const std::string& parent = "chassis",
                   const std::string& parentLink = "root") {
        const auto* info = cat.findTemplate(type);
        sdk::MechanismConfig m;
        m.id = id;
        m.type = type;
        m.label = label;
        m.geometry = info->defaultGeometry;
        m.properties = sdk::defaultsOf(info->properties);
        m.attachTo(parent, parentLink);
        m.mount.position = mount;
        c.mechanisms.push_back(std::move(m));
        return &c.mechanisms.back();
    };
    auto* intake = add("sample.intake", "intake", "Intake", {0.50f, 0.f, -0.06f});
    intake->mappings.push_back({{"intake", "motor"}, {"phoenix6", "talonfx", 10, "canivore", -1}});
    auto* indexer = add("mechanism", "indexer", "Indexer", {0.05f, 0.f, -0.04f});
    indexer->geometry = {prim("belt", sdk::PrimitiveKind::Box, {0.f, 0.f, 0.f}, {0.3f, 0.12f, 0.05f}, "#6aa6d8")};
    auto* turret = add("mechanism", "turret", "Turret", {-0.15f, 0.f, 0.0f});
    turret->properties["minPosition"] = -3.14;
    turret->properties["maxPosition"] = 3.14;
    turret->properties["gearRatio"] = 40.0;
    turret->geometry = {prim("ring", sdk::PrimitiveKind::Cylinder, {0.f, 0.f, 0.02f}, {}, "#7e8fa3", 0.12f, 0.04f)};
    auto* shooter = add("sample.shooter", "shooter", "Shooter", {0.02f, 0.f, 0.06f}, "turret", "mover");
    shooter->mappings.push_back({{"shooter", "flywheel"}, {"phoenix6", "talonfx", 11, "canivore", -1}});
    auto* climber = add("mechanism", "climber", "Climber", {-0.33f, 0.28f, -0.05f});
    climber->properties["motion"] = "linear";
    climber->properties["maxPosition"] = 0.6;
    climber->geometry = {prim("rail", sdk::PrimitiveKind::Box, {0.f, 0.f, 0.3f}, {0.05f, 0.1f, 0.6f}, "#9aa5b1"),
                         prim("carriage", sdk::PrimitiveKind::Box, {0.04f, 0.f, 0.05f}, {0.06f, 0.12f, 0.08f}, "#e9a23b")};

    // Zones the game plugin owns: a pickup volume in front of the intake, and the muzzle riding on the turret.
    const auto condition = [](const char* mechanism, double threshold) {
        sdk::ZoneCondition cond;
        cond.mechanismId = mechanism;
        cond.threshold = threshold;
        return cond;
    };
    sdk::ZoneConfig pickup;
    pickup.id = "pickup";
    pickup.kind = "game_2026.pickup";
    pickup.label = "Pickup";
    pickup.attachment.parentMechanismId = "intake";
    pickup.local.position = {0.25f, 0.f, 0.f};
    pickup.sizeM = {0.4f, 0.6f, 0.25f};
    pickup.properties = sdk::defaultsOf(cat.findZoneKind(pickup.kind)->properties);
    pickup.activation.conditions = {condition("intake", 20.0)};
    c.zones.push_back(std::move(pickup));

    sdk::ZoneConfig muzzle;
    muzzle.id = "muzzle";
    muzzle.kind = "game_2026.shooter";
    muzzle.label = "Shooter muzzle";
    muzzle.attachment.parentMechanismId = "turret";
    muzzle.attachment.parentLink = "mover";
    muzzle.local.position = {0.12f, 0.f, 0.06f};
    muzzle.properties = sdk::defaultsOf(cat.findZoneKind(muzzle.kind)->properties);
    muzzle.activation.conditions = {condition("shooter", 30.0), condition("indexer", 5.0)};
    c.zones.push_back(std::move(muzzle));

    // Vision cameras: a Limelight on the chassis and a PhotonVision camera riding on the turret.
    sdk::CameraConfig limelight;
    limelight.id = "limelight";
    limelight.label = "Limelight";
    limelight.kind = sdk::CameraKind::Limelight;
    limelight.local.position = {0.33f, 0.f, 0.15f};
    c.cameras.push_back(std::move(limelight));

    sdk::CameraConfig photon;
    photon.id = "photon";
    photon.label = "Turret camera";
    photon.name = "photon";
    photon.kind = sdk::CameraKind::PhotonVision;
    photon.attachment.parentMechanismId = "turret";
    photon.attachment.parentLink = "mover";
    photon.local.position = {0.1f, 0.f, 0.1f};
    c.cameras.push_back(std::move(photon));

    // Game-fed sensors: a distance sensor and a beam break, both driven by values the plugin publishes.
    sdk::SensorConfig hopper;
    hopper.id = "hopper_sensor";
    hopper.label = "Hopper sensor";
    hopper.kind = sdk::SensorKind::Distance;
    hopper.source = "game_2026.fuel.count";
    for (const auto& p : sdk::sensorResponseSchema(hopper.kind).properties) hopper.properties[p.key] = p.defaultValue;
    hopper.properties[sdk::sensor_prop::kThreshold] = 1.0;
    c.sensors.push_back(std::move(hopper));

    sdk::SensorConfig feed;
    feed.id = "feed_sensor";
    feed.label = "Feed beam break";
    feed.kind = sdk::SensorKind::BeamBreak;
    feed.source = "game_2026.fuel.fraction";
    for (const auto& p : sdk::sensorResponseSchema(feed.kind).properties) feed.properties[p.key] = p.defaultValue;
    c.sensors.push_back(std::move(feed));
    return c;
}

std::vector<sdk::DiscoveredDevice> FakeEditorBackend::sampleDevices() {
    std::vector<sdk::DiscoveredDevice> out;
    for (int id = 1; id <= 8; ++id) {
        auto d = makeDevice("phoenix6", "talonfx", id, sdk::kBusUnknown, kTalon);
        d.observedBus = "canivore";
        d.mapped = true;
        out.push_back(d);
    }
    auto pigeon = makeDevice("phoenix6", "pigeon2", 13, sdk::kBusUnknown, Capability::Imu);
    pigeon.observedBus = "canivore";
    pigeon.mapped = true;
    out.push_back(pigeon);
    return out;
}

FakeEditorBackend::FakeEditorBackend(QObject* parent) : IEditorBackend(parent) {
    qRegisterMetaType<sdk::DiscoveredDevice>("wpiemu::sdk::DiscoveredDevice");
    qRegisterMetaType<std::vector<sdk::Issue>>("std::vector<wpiemu::sdk::Issue>");
    qRegisterMetaType<sdk::Severity>("wpiemu::sdk::Severity");
    config_ = sampleConfig();
    devices_ = sampleDevices();
    catalog_ = sampleCatalog();
    setCatalog(catalog_);

    auto motorCtl = [](int id, const char* bus) {
        auto d = makeDevice("phoenix6", "talonfx", id, sdk::kBusUnknown, kTalon);
        d.observedBus = bus;
        return d;
    };
    script_.push_back(motorCtl(10, "canivore"));
    script_.push_back(motorCtl(11, "canivore"));
    script_.push_back(makeDevice("phoenix6", "cancoder", 21, sdk::kBusUnknown, Capability::PositionSensor | Capability::VelocitySensor));
    script_.push_back(makeDevice("hal", "dio", 0, "rio", Capability::DigitalInput | Capability::DigitalOutput, "wpilib"));
    script_.push_back(motorCtl(14, "rio"));
    auto aliased = motorCtl(19, "canivore");
    aliased.aliasState = sdk::AliasState::Conflict;
    aliased.aliases = {{"phoenix6", "talonfx", 19, "rio", -1}, {"phoenix6", "talonfx", 19, "canivore", -1}};
    script_.push_back(aliased);
    script_.push_back(makeDevice("phoenix6", "candle", 30, sdk::kBusUnknown, Capability::Led));

    timer_.setSingleShot(false);
    connect(&timer_, &QTimer::timeout, this, [this] { detectionStep(); });
}

void FakeEditorBackend::setConfig(sdk::RobotConfig config) {
    config_ = std::move(config);
    Q_EMIT configReplaced();
}

void FakeEditorBackend::setCatalog(EditorCatalog catalog) {
    catalog_ = std::move(catalog);
    factory_ = sdk::MechanismFactory{};
    for (const auto& t : catalog_.mechanismTemplates)
        (void)factory_.registerTemplate(t, [](const sdk::MechanismBuildContext&) { return std::unique_ptr<sdk::IMechanism>(); });
    Q_EMIT catalogChanged();
}

void FakeEditorBackend::setDevices(std::vector<sdk::DiscoveredDevice> devices) {
    devices_ = std::move(devices);
    Q_EMIT devicesChanged();
}

void FakeEditorBackend::announce(const sdk::DiscoveredDevice& device) {
    for (auto& d : devices_)
        if (d.descriptor.physicalId() == device.descriptor.physicalId()) {
            d = device;
            Q_EMIT deviceUpdated(d);
            return;
        }
    devices_.push_back(device);
    Q_EMIT deviceDiscovered(device);
}

std::vector<sdk::Issue> FakeEditorBackend::validate(const sdk::RobotConfig& c) const {
    std::vector<sdk::Issue> out;
    for (std::size_t i = 0; i < c.mechanisms.size(); ++i) {
        const auto& m = c.mechanisms[i];
        const std::string base = "mechanisms[" + std::to_string(i) + "]";
        const auto* info = catalog_.findTemplate(m.type);
        if (info == nullptr) {
            out.push_back({sdk::Severity::Error, base + ".type", "unknown mechanism type '" + m.type + "'", {}});
            continue;
        }
        for (auto& issue : sdk::validateProperties(info->properties, m.properties, base + ".properties"))
            if (issue.severity == sdk::Severity::Error || issue.message != "unknown property") out.push_back(std::move(issue));
    }
    for (auto& issue : sdk::validateMechanismAttachments(c, factory_)) out.push_back(std::move(issue));
    for (auto& issue : validateZones(c)) out.push_back(std::move(issue));
    for (auto& issue : validateCameras(c)) out.push_back(std::move(issue));
    for (auto& issue : validateSensors(c)) out.push_back(std::move(issue));
    for (auto& issue : sdk::validateDeviceAliases(c.allMappings())) out.push_back(std::move(issue));
    if (c.chassis.massKg < 10.f) out.push_back({sdk::Severity::Warning, "chassis.massKg", "chassis mass below 10 kg is unusual for an FRC robot", {}});
    return out;
}

std::vector<sdk::Issue> FakeEditorBackend::validateZones(const sdk::RobotConfig& c) const {
    std::vector<sdk::Issue> out;
    for (std::size_t i = 0; i < c.zones.size(); ++i) {
        const sdk::ZoneConfig& z = c.zones[i];
        const std::string base = "zones[" + std::to_string(i) + "]";
        if (z.id.empty() || c.findMechanism(z.id) != nullptr || c.findCamera(z.id) != nullptr)
            out.push_back({sdk::Severity::Error, base + ".id", "zone id must be unique and not a mechanism id", "zone_id_duplicate"});
        const sdk::MechanismAttachment att = z.attachment;
        if (!att.onChassis()) {
            const sdk::MechanismConfig* parent = c.findMechanism(att.parentMechanismId);
            const sdk::MechanismTemplateInfo* info = parent != nullptr ? catalog_.findTemplate(parent->type) : nullptr;
            if (parent == nullptr)
                out.push_back({sdk::Severity::Error, base + ".attachment", "parent mechanism '" + att.parentMechanismId + "' does not exist",
                               sdk::validation_code::kAttachmentParentMissing});
            else if (info != nullptr && !sdk::hasLink(*info, att.parentLink))
                out.push_back({sdk::Severity::Error, base + ".attachment", "mechanism '" + parent->id + "' has no link '" + att.parentLink + "'",
                               sdk::validation_code::kAttachmentLinkMissing});
        } else if (att.parentLink != sdk::kRootLink) {
            out.push_back({sdk::Severity::Error, base + ".attachment", "the chassis only exposes the link 'root'", sdk::validation_code::kAttachmentLinkMissing});
        }
        const sdk::ZoneKindInfo* kind = catalog_.findZoneKind(z.kind);
        if (kind == nullptr)
            out.push_back({sdk::Severity::Warning, base + ".kind", "unknown zone kind '" + z.kind + "'", "zone_kind_unknown"});
        else
            for (auto& issue : sdk::validateProperties(kind->properties, z.properties, base + ".properties"))
                if (issue.severity == sdk::Severity::Error || issue.message != "unknown property") {
                    if (issue.code.empty()) issue.code = "zone_property_invalid";
                    out.push_back(std::move(issue));
                }
        for (std::size_t j = 0; j < z.activation.conditions.size(); ++j)
            if (c.findMechanism(z.activation.conditions[j].mechanismId) == nullptr)
                out.push_back({sdk::Severity::Warning, base + ".activation.conditions[" + std::to_string(j) + "].mechanism",
                               "condition mechanism '" + z.activation.conditions[j].mechanismId + "' does not exist", "zone_condition_mechanism_unknown"});
    }
    return out;
}

std::vector<sdk::Issue> FakeEditorBackend::validateSensors(const sdk::RobotConfig& c) const {
    // Mirrors the real validator's sensor source checks (core: config_validator.h, issue_code::kSensor*).
    std::vector<sdk::Issue> out;
    for (std::size_t i = 0; i < c.sensors.size(); ++i) {
        const sdk::SensorConfig& s = c.sensors[i];
        const std::string base = "sensors[" + std::to_string(i) + "]";
        if (s.source.empty())
            out.push_back({sdk::Severity::Warning, base + ".source", "sensor has no source; it reads as idle", "sensor_source_missing"});
        else if (catalog_.findGameValue(s.source) == nullptr)
            out.push_back({sdk::Severity::Warning, base + ".source", "the game plugin does not publish '" + s.source + "'", "sensor_source_unknown"});
    }
    return out;
}

std::vector<sdk::Issue> FakeEditorBackend::validateCameras(const sdk::RobotConfig& c) const {
    // Mirrors the real validator's camera checks (core: config_validator.h, issue_code::kCamera*).
    std::vector<sdk::Issue> out;
    auto finite = [](double v) { return std::isfinite(v); };
    for (std::size_t i = 0; i < c.cameras.size(); ++i) {
        const sdk::CameraConfig& cam = c.cameras[i];
        const std::string base = "cameras[" + std::to_string(i) + "]";
        if (cam.id.empty())
            out.push_back({sdk::Severity::Error, base + ".id", "camera id must not be empty", "camera_id_invalid"});
        else if (c.findMechanism(cam.id) != nullptr || c.findZone(cam.id) != nullptr ||
                 std::count_if(c.cameras.begin(), c.cameras.end(), [&](const sdk::CameraConfig& o) { return o.id == cam.id; }) > 1)
            out.push_back({sdk::Severity::Error, base + ".id", "camera id must be unique and not a mechanism or zone id", "camera_id_duplicate"});
        if (std::any_of(c.cameras.begin(), c.cameras.begin() + static_cast<std::ptrdiff_t>(i),
                        [&](const sdk::CameraConfig& o) { return o.kind == cam.kind && o.name == cam.name; }))
            out.push_back({sdk::Severity::Error, base + ".name", "another camera of this type already uses the name '" + cam.name + "'",
                           "camera_name_duplicate"});
        const sdk::MechanismAttachment att = cam.attachment;
        if (!att.onChassis()) {
            const sdk::MechanismConfig* parent = c.findMechanism(att.parentMechanismId);
            const sdk::MechanismTemplateInfo* info = parent != nullptr ? catalog_.findTemplate(parent->type) : nullptr;
            if (parent == nullptr)
                out.push_back({sdk::Severity::Error, base + ".attachment", "parent mechanism '" + att.parentMechanismId + "' does not exist",
                               sdk::validation_code::kAttachmentParentMissing});
            else if (info != nullptr && !sdk::hasLink(*info, att.parentLink))
                out.push_back({sdk::Severity::Error, base + ".attachment", "mechanism '" + parent->id + "' has no link '" + att.parentLink + "'",
                               sdk::validation_code::kAttachmentLinkMissing});
        } else if (att.parentLink != sdk::kRootLink) {
            out.push_back({sdk::Severity::Error, base + ".attachment", "the chassis only exposes the link 'root'", sdk::validation_code::kAttachmentLinkMissing});
        }
        if (!sdk::finite(cam.local))
            out.push_back({sdk::Severity::Error, base + ".local", "camera pose must be finite", "camera_geometry_invalid"});
        auto optics = [&](const char* key, bool ok, const char* what) {
            if (!ok) out.push_back({sdk::Severity::Error, base + "." + key, what, "camera_optics_invalid"});
        };
        optics("hFovDeg", finite(cam.hFovDeg) && cam.hFovDeg > 0.0 && cam.hFovDeg < 180.0, "horizontal field of view must be in (0, 180) degrees");
        optics("vFovDeg", finite(cam.vFovDeg) && cam.vFovDeg > 0.0 && cam.vFovDeg < 180.0, "vertical field of view must be in (0, 180) degrees");
        optics("maxRangeM", finite(cam.maxRangeM) && cam.maxRangeM > 0.0, "max range must be positive");
        optics("maxIncidenceDeg", finite(cam.maxIncidenceDeg) && cam.maxIncidenceDeg > 0.0 && cam.maxIncidenceDeg <= 90.0,
               "max incidence must be in (0, 90] degrees");
        optics("fps", finite(cam.fps) && cam.fps > 0.0, "frame rate must be positive");
        optics("latencyMs", finite(cam.latencyMs) && cam.latencyMs >= 0.0, "latency must not be negative");
        optics("widthPx", cam.widthPx > 0, "resolution must be positive");
        optics("heightPx", cam.heightPx > 0, "resolution must be positive");
        optics("positionNoiseM", finite(cam.positionNoiseM) && cam.positionNoiseM >= 0.0, "noise must not be negative");
        optics("yawNoiseDeg", finite(cam.yawNoiseDeg) && cam.yawNoiseDeg >= 0.0, "noise must not be negative");
    }
    return out;
}

void FakeEditorBackend::submitConfig(const sdk::RobotConfig& config) {
    config_ = config;
    ++submitCount_;
}

sdk::Status FakeEditorBackend::saveConfig() {
    ++saveCount_;
    if (!saveError_.isEmpty())
        return sdk::makeError(sdk::ErrorCode::IoError, saveError_.toStdString());
    Q_EMIT logMessage(sdk::Severity::Info, QStringLiteral("Saved robot.json"));
    return sdk::okStatus();
}

void FakeEditorBackend::startDetection() {
    if (detecting_) return;
    detecting_ = true;
    scriptIndex_ = 0;
    Q_EMIT detectionStateChanged(true);
    Q_EMIT logMessage(sdk::Severity::Info, QStringLiteral("Discovery started: launching the robot in discovery mode..."));
    timer_.start(stepMs_);
}

void FakeEditorBackend::stopDetection() {
    if (!detecting_) return;
    detecting_ = false;
    timer_.stop();
    Q_EMIT detectionStateChanged(false);
    Q_EMIT logMessage(sdk::Severity::Info, QStringLiteral("Discovery stopped."));
}

void FakeEditorBackend::detectionStep() {
    if (scriptIndex_ >= script_.size()) {
        stopDetection();
        return;
    }
    const auto& d = script_[scriptIndex_++];
    Q_EMIT logMessage(sdk::Severity::Info, QStringLiteral("[robot] announced %1 %2").arg(QString::fromStdString(d.descriptor.type)).arg(d.descriptor.id));
    announce(d);
}

}  // namespace wpiemu::app::editor
