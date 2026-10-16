#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

#include "wpiemu/core/mechanisms/motor_drive.h"
#include "wpiemu/core/mechanisms/sensor_models.h"
#include "wpiemu/sdk/core/units.h"

using namespace wpiemu::core::mechanisms;
namespace sdk = wpiemu::sdk;

namespace {

sdk::MotorSpec neo() { return *MotorCatalog::builtin().spec("neo"); }

sdk::MotorOutput volts(double v) {
    sdk::MotorOutput o;
    o.valid = true;
    o.voltageV = v;
    return o;
}

}  // namespace

TEST_CASE("motor drive passes voltage commands through, saturating at the battery voltage") {
    MotorDrive drive(neo(), 0.0, NeutralMode::Brake);
    CHECK(drive.voltage(volts(6.0), 0.0, 12.0) == 6.0);
    CHECK(drive.voltage(volts(-6.0), 0.0, 12.0) == -6.0);
    CHECK(drive.voltage(volts(12.0), 0.0, 9.0) == 9.0);      // sag
    CHECK(drive.voltage(volts(-12.0), 0.0, 9.0) == -9.0);
    CHECK(drive.voltage(volts(12.0), 0.0, 0.0) == 0.0);      // dead battery
}

TEST_CASE("motor drive derives voltage from duty cycle with the present battery voltage") {
    MotorDrive drive(neo(), 0.0, NeutralMode::Brake);
    sdk::MotorOutput duty;
    duty.valid = true;
    duty.dutyCycle = 0.5;
    CHECK(drive.voltage(duty, 0.0, 11.0) == doctest::Approx(5.5));
    duty.dutyCycle = -1.0;
    CHECK(drive.voltage(duty, 0.0, 10.0) == doctest::Approx(-10.0));
    duty.dutyCycle = 1.0;
    duty.voltageV = 3.0;   // an explicit voltage wins over duty
    CHECK(drive.voltage(duty, 0.0, 12.0) == 3.0);
}

TEST_CASE("neutral output depends on the neutral mode") {
    const sdk::MotorSpec m = neo();
    const double omega = 200.0;
    const double backEmf = omega / m.kvRadPerSecPerVolt();
    MotorDrive brake(m, 0.0, NeutralMode::Brake);
    MotorDrive coast(m, 0.0, NeutralMode::Coast);
    CHECK(brake.voltage(volts(0.0), omega, 12.0) == 0.0);
    CHECK(coast.voltage(volts(0.0), omega, 12.0) == doctest::Approx(backEmf));
    CHECK(coast.currentA(coast.voltage(volts(0.0), omega, 12.0), omega) == doctest::Approx(0.0).epsilon(1e-9));
    // no data from the device (unmapped slot) is neutral as well
    CHECK(brake.voltage(sdk::MotorOutput{}, omega, 12.0) == 0.0);
    CHECK(coast.voltage(sdk::MotorOutput{}, omega, 12.0) == doctest::Approx(backEmf));
}

TEST_CASE("torque-current requests become I*R + back-EMF and respect the supply") {
    const sdk::MotorSpec m = neo();
    MotorDrive drive(m, 0.0, NeutralMode::Brake);
    sdk::MotorOutput out;
    out.valid = true;
    out.hasTorqueCurrent = true;
    out.torqueCurrentA = 20.0;
    const double omega = 150.0;
    const double v = drive.voltage(out, omega, 12.0);
    CHECK(v == doctest::Approx(20.0 * m.resistanceOhm() + omega / m.kvRadPerSecPerVolt()));
    CHECK(drive.currentA(v, omega) == doctest::Approx(20.0).epsilon(1e-9));
    out.torqueCurrentA = 1000.0;   // not reachable: saturates at the battery
    CHECK(drive.voltage(out, omega, 12.0) == 12.0);
    out.torqueCurrentA = 0.0;
    CHECK(drive.voltage(out, omega, 12.0) == 0.0);   // brake neutral
}

TEST_CASE("stator current limit caps the motor current at any speed") {
    const sdk::MotorSpec m = neo();
    MotorDrive drive(m, 30.0, NeutralMode::Brake);
    for (const double omega : {0.0, 100.0, 300.0}) {
        const double v = drive.voltage(volts(12.0), omega, 12.0);
        const double i = drive.currentA(v, omega);
        CHECK(i <= 30.0 + 1e-9);
        const double unlimited = (12.0 - omega / m.kvRadPerSecPerVolt()) / m.resistanceOhm();
        CHECK(i == doctest::Approx(std::min(30.0, unlimited)).epsilon(1e-9));
    }
    // regenerative / reverse current is limited too
    const double v = drive.voltage(volts(-12.0), 0.0, 12.0);
    CHECK(drive.currentA(v, 0.0) == doctest::Approx(-30.0));
}

TEST_CASE("encoder model applies ratio, inversion and offset") {
    EncoderModel e;
    e.sensorRatio = 4.0;
    e.offsetRot = 0.25;
    sdk::EncoderFeedback f = e.sample(0.5, 2.0);
    CHECK(f.positionRot == doctest::Approx(2.25));
    CHECK(f.velocityRps == doctest::Approx(8.0));
    e.inverted = true;
    f = e.sample(0.5, 2.0);
    CHECK(f.positionRot == doctest::Approx(-1.75));
    CHECK(f.velocityRps == doctest::Approx(-8.0));
}

TEST_CASE("rotor sensor model is on the motor side of the gearbox") {
    RotorSensorModel r;
    r.gearRatio = 12.0;
    sdk::MotorFeedback f = r.sample(0.1, 0.5, 2.0);
    CHECK(f.rotorPositionRot == doctest::Approx(1.2));
    CHECK(f.rotorVelocityRps == doctest::Approx(6.0));
    CHECK(f.rotorAccelerationRps2 == doctest::Approx(24.0));
    CHECK_FALSE(f.forwardLimit.has_value());
    r.inverted = true;
    f = r.sample(0.1, 0.5, 2.0);
    CHECK(f.rotorPositionRot == doctest::Approx(-1.2));
}

TEST_CASE("limit switch and window beam break models honour sides, margin and polarity") {
    LimitSwitchModel upper{1.0, LimitSwitchModel::Side::AtOrAbove, 0.01, true};
    CHECK_FALSE(upper.pressed(0.98));
    CHECK(upper.pressed(0.99));
    CHECK(upper.pressed(1.0));
    CHECK(upper.reading(1.0));
    upper.pressedValue = false;   // normally closed
    CHECK(upper.reading(0.0));
    CHECK_FALSE(upper.reading(1.0));

    LimitSwitchModel lower{0.0, LimitSwitchModel::Side::AtOrBelow, 0.0, true};
    CHECK(lower.pressed(0.0));
    CHECK(lower.pressed(-1.0));
    CHECK_FALSE(lower.pressed(1e-3));

    WindowBeamBreakModel beam{0.2, 0.4, true};
    CHECK(beam.reading(0.3));
    CHECK(beam.reading(0.2));
    CHECK_FALSE(beam.reading(0.41));
    beam.blockedValue = false;
    CHECK_FALSE(beam.reading(0.3));
    CHECK(beam.reading(0.0));
}
