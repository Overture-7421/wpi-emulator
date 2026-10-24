#include <functional>
#include <utility>

#include "can_bridge.h"
#include "digital_bridges.h"
#include "led_bridge.h"
#include "pneumatics_bridges.h"
#include "power_bridges.h"
#include "sensor_bridges.h"
#include "wpiemu/bridges/hal/hal_bridges.h"

namespace wpiemu::bridges::hal {

namespace {

// A shared_ptr to one sub-interface of the aggregate that keeps the whole aggregate alive.
template <class Sub>
std::shared_ptr<Sub> share(const std::shared_ptr<IHalApi>& api, Sub& sub) {
    return std::shared_ptr<Sub>(api, &sub);
}

class Registrar {
public:
    Registrar(sdk::DeviceBridgeRegistry& registry, std::shared_ptr<IHalApi> api)
        : registry_(registry), api_(std::move(api)) {}

    // `make` receives the shared api and builds the bridge; it is stored in the registry factory.
    template <class Make>
    void add(std::string name, const char* description, const char* vendor, Make make) {
        if (!status_.ok()) return;
        std::shared_ptr<IHalApi> api = api_;
        status_ = registry_.add(
            std::move(name), sdk::BridgeInfo{"hal", description, vendor},
            [api, make]() -> std::unique_ptr<sdk::IDeviceBridge> { return make(api); });
    }

    sdk::Status status() const { return status_; }

private:
    sdk::DeviceBridgeRegistry& registry_;
    std::shared_ptr<IHalApi> api_;
    sdk::Status status_ = sdk::okStatus();
};

}  // namespace

sdk::Status registerHalBridges(sdk::DeviceBridgeRegistry& registry, std::shared_ptr<IHalApi> api, HalBridgeOptions options) {
    if (!api) return sdk::makeError(sdk::ErrorCode::InvalidArgument, "registerHalBridges: null IHalApi");

    const bool analogOut = api->analogOut() != nullptr;
    const bool relay = api->relay() != nullptr;
    const bool accelerometer = api->accelerometer() != nullptr;
    const bool imu = api->imu() != nullptr;
    Registrar r(registry, std::move(api));
    const HalBridgeOptions o = options;

    r.add("hal.pwm", "PWM outputs (speed controllers, servos)", "wpilib",
          [](const std::shared_ptr<IHalApi>& a) { return std::make_unique<PwmBridge>(share(a, a->pwm())); });
    r.add("hal.dio", "Digital inputs (limit switches, beam breaks) and outputs", "wpilib",
          [](const std::shared_ptr<IHalApi>& a) { return std::make_unique<DioBridge>(share(a, a->dio())); });
    r.add("hal.analog_in", "Analog inputs", "wpilib",
          [](const std::shared_ptr<IHalApi>& a) { return std::make_unique<AnalogInBridge>(share(a, a->analogIn())); });
    if (analogOut) r.add("hal.analog_out", "Analog outputs (observe)", "wpilib",
          [](const std::shared_ptr<IHalApi>& a) { return std::make_unique<AnalogOutBridge>(share(a, *a->analogOut())); });
    r.add("hal.encoder", "Quadrature encoders (A/B pairs)", "wpilib", [o](const std::shared_ptr<IHalApi>& a) {
        return std::make_unique<EncoderBridge>(share(a, a->encoder()), o.encoderCountsPerRevolution);
    });
    r.add("hal.duty_cycle", "DutyCycle inputs / absolute encoders", "wpilib", [o](const std::shared_ptr<IHalApi>& a) {
        return std::make_unique<DutyCycleBridge>(share(a, a->dutyCycle()), o.dutyCycleDefaultFrequencyHz);
    });
    if (relay) r.add("hal.relay", "Relay outputs (forward / reverse)", "wpilib",
          [](const std::shared_ptr<IHalApi>& a) { return std::make_unique<RelayBridge>(share(a, *a->relay())); });

    const PneumaticsKind pcm = PneumaticsKind::ctrePcm();
    r.add(pcm.bridgeName, "CTRE PCM modules (compressor, pressure switch)", "ctre", [pcm](const std::shared_ptr<IHalApi>& a) {
        return std::make_unique<PneumaticsModuleBridge>(share(a, a->ctrePcm()), pcm);
    });
    r.add(pcm.bridgeName + ".solenoid", "CTRE PCM solenoid channels", "ctre", [pcm, o](const std::shared_ptr<IHalApi>& a) {
        return std::make_unique<SolenoidBridge>(share(a, a->ctrePcm()), pcm, o.announceAllSolenoidChannels);
    });
    const PneumaticsKind ph = PneumaticsKind::revPh();
    r.add(ph.bridgeName, "REV PH modules (compressor, pressure switch)", "rev", [ph](const std::shared_ptr<IHalApi>& a) {
        return std::make_unique<PneumaticsModuleBridge>(share(a, a->revPh()), ph);
    });
    r.add(ph.bridgeName + ".solenoid", "REV PH solenoid channels", "rev", [ph, o](const std::shared_ptr<IHalApi>& a) {
        return std::make_unique<SolenoidBridge>(share(a, a->revPh()), ph, o.announceAllSolenoidChannels);
    });

    r.add("hal.power_distribution", "PDP / PDH current, voltage, temperature", "wpilib", [](const std::shared_ptr<IHalApi>& a) {
        return std::make_unique<PowerDistributionBridge>(share(a, a->powerDistribution()));
    });
    r.add("hal.addressable_led", "Addressable LED strips (observe)", "wpilib", [](const std::shared_ptr<IHalApi>& a) {
        return std::make_unique<AddressableLedBridge>(share(a, a->addressableLed()));
    });
    r.add("hal.roborio", "roboRIO battery voltage / current and brownout threshold", "wpilib",
          [](const std::shared_ptr<IHalApi>& a) { return std::make_unique<RoboRioBridge>(share(a, a->roboRio())); });
    if (accelerometer) r.add("hal.accelerometer", "roboRIO built-in accelerometer", "wpilib", [](const std::shared_ptr<IHalApi>& a) {
        return std::make_unique<AccelerometerBridge>(share(a, *a->accelerometer()));
    });
    if (imu) r.add("hal.imu", "Onboard IMU", "wpilib", [](const std::shared_ptr<IHalApi>& a) {
        return std::make_unique<ImuBridge>(share(a, *a->imu()));
    });
    r.add("hal.can", "Generic HAL CAN frames (observe only)", "wpilib",
          [](const std::shared_ptr<IHalApi>& a) { return std::make_unique<CanFrameBridge>(share(a, a->can())); });

    return r.status();
}

}  // namespace wpiemu::bridges::hal
