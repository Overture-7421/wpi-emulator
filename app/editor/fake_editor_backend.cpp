#include "fake_editor_backend.h"

#include <algorithm>

#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/devices/device_identity.h"
#include "wpiemu/sdk/mechanisms/attachment.h"

namespace wpiemu::app::editor {

using sdk::Capability;
using sdk::CapabilitySet;
using sdk::PropertySpec;
using sdk::PropertyType;
using sdk::PropertyUnit;

namespace {

PropertySpec num(const char* key, const char* label, const char* group, PropertyUnit unit, double def, double min, double max,
                 double step = 0.0, const char* description = "") {
    PropertySpec s;
    s.key = key;
    s.label = label;
    s.group = group;
    s.unit = unit;
    s.type = PropertyType::Double;
    s.defaultValue = def;
    s.min = min;
    s.max = max;
    s.step = step;
    s.description = description;
    return s;
}

PropertySpec integer(const char* key, const char* label, const char* group, int def, int min, int max, const char* description = "") {
    PropertySpec s;
    s.key = key;
    s.label = label;
    s.group = group;
    s.unit = PropertyUnit::Count;
    s.type = PropertyType::Int;
    s.defaultValue = def;
    s.min = min;
    s.max = max;
    s.description = description;
    return s;
}

PropertySpec flag(const char* key, const char* label, const char* group, bool def, const char* description = "") {
    PropertySpec s;
    s.key = key;
    s.label = label;
    s.group = group;
    s.type = PropertyType::Bool;
    s.defaultValue = def;
    s.description = description;
    return s;
}

PropertySpec choice(const char* key, const char* label, const char* group, const char* def,
                    std::vector<sdk::EnumOption> options, const char* description = "") {
    PropertySpec s;
    s.key = key;
    s.label = label;
    s.group = group;
    s.type = PropertyType::Enum;
    s.defaultValue = def;
    s.options = std::move(options);
    s.description = description;
    return s;
}

PropertySpec volume(const char* key, const char* label, const char* group, const sdk::Box3& def, const char* description = "") {
    PropertySpec s;
    s.key = key;
    s.label = label;
    s.group = group;
    s.type = PropertyType::Box3;
    s.unit = PropertyUnit::Meters;
    s.defaultValue = sdk::toValue(def);
    s.description = description;
    return s;
}

PropertySpec intCount(const char* key, const char* label, const char* group, int def, int max) {
    PropertySpec p = num(key, label, group, PropertyUnit::Count, def, 1.0, max, 1.0);
    p.type = PropertyType::Int;
    p.defaultValue = def;
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
    return {{"Kraken X60", "Kraken X60"}, {"Falcon 500", "Falcon 500"}, {"NEO", "NEO"}, {"NEO 550", "NEO 550"},
            {"NEO Vortex", "NEO Vortex"}, {"CIM", "CIM"}, {"MiniCIM", "MiniCIM"}};
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
         num("gearRatio", "Gear ratio", kMotor, PropertyUnit::Ratio, 10.0, 0.1, 500.0, 0.5),
         num("inertiaKgM2", "Moment of inertia", kMech, PropertyUnit::KgM2, 0.01, 0.0001, 10.0, 0.001),
         num("massKg", "Mass", kMech, PropertyUnit::Kilograms, 2.0, 0.01, 100.0, 0.1),
         num("minPosition", "Minimum position", kMech, PropertyUnit::None, 0.0, -10.0, 10.0, 0.01),
         num("maxPosition", "Maximum position", kMech, PropertyUnit::None, 0.0, -10.0, 10.0, 0.01)},
        {prim("mover", sdk::PrimitiveKind::Box, {0.10f, 0.f, 0.f}, {0.20f, 0.05f, 0.05f}, "#9b59b6")},
        {link("mover", "Moving part", {0.f, 0.f, 0.05f})}));
    // ---- plugin templates (prefix = plugin id) with the pickup-zone volume ----
    sdk::Box3 zone;
    zone.pose.position = {0.3f, 0.f, 0.f};
    zone.sizeM = {0.4f, 0.6f, 0.25f};
    cat.mechanismTemplates.push_back(templ(
        "game_2026.fuel_intake", "Fuel intake", "Intake", "Intake that absorbs field fuel inside its pickup zone while running.",
        {slot("motor", "Roller motor", sdk::SlotType::Motor, true), slot("deploy", "Deploy motor", sdk::SlotType::Motor, false),
         slot("deployEncoder", "Deploy encoder", sdk::SlotType::PositionSensor, false)},
        {volume("pickupZone", "Pickup zone", "Fuel", zone, "Fuel whose centre is inside this volume is absorbed while the intake runs."),
         integer("capacity", "Capacity (fuel)", "Fuel", 30, 1, 100), num("gearRatio", "Gear ratio", kMotor, PropertyUnit::Ratio, 3.0, 0.5, 60.0, 0.1),
         choice("motorType", "Motor", kMotor, "NEO", motorChoices()),
         num("activeThresholdRpm", "Active above", "Fuel", PropertyUnit::RadiansPerSecond, 20.0, 0.0, 600.0, 1.0)},
        {prim("roller", sdk::PrimitiveKind::Cylinder, {0.f, 0.f, 0.f}, {}, "#ffb347", 0.03f, 0.55f, true),
         prim("frame", sdk::PrimitiveKind::Box, {-0.06f, 0.f, 0.02f}, {0.1f, 0.55f, 0.05f}, "#9aa5b1")},
        {}, "game_2026"));

    cat.mechanismTemplates.push_back(templ(
        "game_2026.fuel_shooter", "Fuel shooter", "Shooter", "Flywheel shooter that spawns fuel when its feeder runs.",
        {slot("flywheel", "Flywheel motor", sdk::SlotType::Motor, true), slot("feeder", "Feeder motor", sdk::SlotType::Motor, true),
         slot("encoder", "Flywheel encoder", sdk::SlotType::VelocitySensor, false)},
        {num("wheelDiameterM", "Wheel diameter", "Flywheel", PropertyUnit::Meters, 0.1016, 0.03, 0.3, 0.005),
         num("efficiency", "Launch efficiency", "Flywheel", PropertyUnit::Ratio, 0.8, 0.1, 1.0, 0.01),
         num("fireRateHz", "Fire rate", "Feeding", PropertyUnit::Hertz, 8.0, 0.5, 30.0, 0.5),
         num("feedThresholdRpm", "Feeding above", "Feeding", PropertyUnit::RadiansPerSecond, 30.0, 0.0, 600.0, 1.0),
         choice("motorType", "Motor", kMotor, "Kraken X60", motorChoices())},
        {prim("flywheel", sdk::PrimitiveKind::Cylinder, {0.f, 0.f, 0.f}, {}, "#d1495b", 0.05f, 0.12f, true),
         prim("housing", sdk::PrimitiveKind::Box, {-0.06f, 0.f, 0.f}, {0.14f, 0.2f, 0.12f}, "#9aa5b1")},
        {link("exit", "Exit point", {0.1f, 0.f, 0.f})}, "game_2026"));

    cat.drivetrains = {{"swerve", {"Swerve", "Independently steered and driven modules."}},
                       {"tank", {"Tank", "Left / right skid steer."}},
                       {"mecanum", {"Mecanum", "Four mecanum wheels."}}};

    // ---- plugin config ----
    cat.pluginId = "game_2026";
    cat.pluginDisplayName = "REBUILT (2026)";
    cat.pluginSchema.schemaVersion = 1;
    cat.pluginSchema.properties.properties = {
        integer("preloadFuel", "Preloaded fuel", "Robot", 8, 0, 100, "Fuel the robot starts a match with."),
        num("fuelDiameterM", "Fuel diameter", "Fuel", PropertyUnit::Meters, 0.15, 0.05, 0.4, 0.005)};
    sdk::Box3 hopper;
    hopper.pose.position = {-0.05f, 0.f, 0.f};
    hopper.sizeM = {0.4f, 0.5f, 0.3f};

    sdk::EditorPanelDescriptor hopperPanel;
    hopperPanel.id = "game_2026.hopper";
    hopperPanel.title = "Fuel hopper";
    hopperPanel.scope = sdk::PanelScope::PluginConfig;
    hopperPanel.properties.properties = {integer("capacity", "Fuel capacity", "Hopper", 40, 1, 200, "The fuel meter shows this as 100%."),
                                         volume("hopperVolume", "Hopper volume", "Hopper", hopper, "Where stored fuel is drawn (chassis frame).")};
    hopperPanel.overlays.push_back({"hopperVolume", "Hopper", "#44cc88", true});
    hopperPanel.order = 1;
    cat.pluginPanels.push_back(hopperPanel);

    // A mechanism-scope panel: only mechanisms of the listed (plugin) templates get it.
    sdk::EditorPanelDescriptor aiming;
    aiming.id = "game_2026.aiming";
    aiming.title = "Aiming";
    aiming.scope = sdk::PanelScope::Mechanism;
    aiming.mechanismTypes = {"game_2026.fuel_shooter"};
    aiming.properties.properties = {num("hoodAngleRad", "Hood angle", "Aiming", PropertyUnit::Radians, 0.7, 0.0, 1.5, 0.01)};
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
    auto* intake = add("game_2026.fuel_intake", "intake", "Intake", {0.50f, 0.f, -0.06f});
    intake->mappings.push_back({{"intake", "motor"}, {"phoenix6", "talonfx", 10, "canivore", -1}});
    auto* indexer = add("mechanism", "indexer", "Indexer", {0.05f, 0.f, -0.04f});
    indexer->geometry = {prim("belt", sdk::PrimitiveKind::Box, {0.f, 0.f, 0.f}, {0.3f, 0.12f, 0.05f}, "#6aa6d8")};
    auto* turret = add("mechanism", "turret", "Turret", {-0.15f, 0.f, 0.0f});
    turret->properties["minPosition"] = -3.14;
    turret->properties["maxPosition"] = 3.14;
    turret->properties["gearRatio"] = 40.0;
    turret->geometry = {prim("ring", sdk::PrimitiveKind::Cylinder, {0.f, 0.f, 0.02f}, {}, "#7e8fa3", 0.12f, 0.04f)};
    auto* shooter = add("game_2026.fuel_shooter", "shooter", "Shooter", {0.02f, 0.f, 0.06f}, "turret", "mover");
    shooter->mappings.push_back({{"shooter", "flywheel"}, {"phoenix6", "talonfx", 11, "canivore", -1}});
    auto* climber = add("mechanism", "climber", "Climber", {-0.33f, 0.28f, -0.05f});
    climber->properties["motion"] = "linear";
    climber->properties["maxPosition"] = 0.6;
    climber->geometry = {prim("rail", sdk::PrimitiveKind::Box, {0.f, 0.f, 0.3f}, {0.05f, 0.1f, 0.6f}, "#9aa5b1"),
                         prim("carriage", sdk::PrimitiveKind::Box, {0.04f, 0.f, 0.05f}, {0.06f, 0.12f, 0.08f}, "#e9a23b")};
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
    for (auto& issue : sdk::validateDeviceAliases(c.allMappings())) out.push_back(std::move(issue));
    if (c.chassis.massKg < 10.f) out.push_back({sdk::Severity::Warning, "chassis.massKg", "chassis mass below 10 kg is unusual for an FRC robot", {}});
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
