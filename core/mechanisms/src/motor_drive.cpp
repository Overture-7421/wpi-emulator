#include "wpiemu/core/mechanisms/motor_drive.h"

#include <algorithm>
#include <cmath>

namespace wpiemu::core::mechanisms {

namespace {
constexpr double kNeutralEps = 1e-9;
}

MotorDrive::MotorDrive(sdk::MotorSpec motor, double statorCurrentLimitA, NeutralMode neutral)
    : model_(std::make_unique<sdk::DcMotorModel>(std::move(motor)), statorCurrentLimitA), neutral_(neutral) {}

double MotorDrive::voltage(const sdk::MotorOutput& out, double omega, double batteryV) const {
    const double backEmf = omega / spec().kvRadPerSecPerVolt();
    double commanded = 0.0;
    bool neutral = !out.valid;
    if (out.valid) {
        if (out.hasTorqueCurrent) {
            commanded = sdk::voltageForTorqueCurrent(spec(), out.torqueCurrentA, omega);
            neutral = std::abs(out.torqueCurrentA) < kNeutralEps;
        } else {
            commanded = out.voltageV;
            if (std::abs(commanded) < kNeutralEps && std::abs(out.dutyCycle) > kNeutralEps)
                commanded = out.dutyCycle * batteryV;
            neutral = std::abs(commanded) < kNeutralEps;
        }
    }
    if (neutral) return neutral_ == NeutralMode::Coast ? backEmf : 0.0;

    const double supply = std::max(batteryV, 0.0);
    commanded = std::clamp(commanded, -supply, supply);
    return model_.limitedVoltage(commanded, omega);
}

}  // namespace wpiemu::core::mechanisms
