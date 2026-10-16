#include <doctest/doctest.h>

#include <cmath>

#include "wpiemu/core/mechanisms/linear_system.h"
#include "wpiemu/core/mechanisms/motor_catalog.h"
#include "wpiemu/core/mechanisms/plants.h"
#include "wpiemu/sdk/core/units.h"

using namespace wpiemu::core::mechanisms;
namespace sdk = wpiemu::sdk;

namespace {

constexpr double kDt = 0.005;

sdk::PlantFactory makeFactory() {
    sdk::PlantFactory f;
    REQUIRE(registerBuiltinPlants(f).ok());
    return f;
}

std::unique_ptr<sdk::IPlant> makePlant(const sdk::PlantSpec& spec) {
    const sdk::PlantFactory f = makeFactory();
    auto p = f.createFor(spec);
    REQUIRE(p.ok());
    return std::move(p).value();
}

sdk::PlantSpec neoSpec(const char* model, int count = 2) {
    sdk::PlantSpec s;
    s.model = model;
    s.motor = *MotorCatalog::builtin().spec("neo");
    s.motorCount = count;
    return s;
}

// Independent reference: dx = f(x) integrated with tiny RK4 steps. f(q, v) -> accel.
template <class F>
sdk::PlantState integrate(sdk::PlantState x, double seconds, double h, F&& accel) {
    const int n = static_cast<int>(std::lround(seconds / h));
    for (int i = 0; i < n; ++i) {
        const double k1q = x.velocity, k1v = accel(x.position, x.velocity);
        const double k2q = x.velocity + 0.5 * h * k1v, k2v = accel(x.position + 0.5 * h * k1q, k2q);
        const double k3q = x.velocity + 0.5 * h * k2v, k3v = accel(x.position + 0.5 * h * k2q, k3q);
        const double k4q = x.velocity + h * k3v, k4v = accel(x.position + h * k3q, k4q);
        x.position += h / 6.0 * (k1q + 2 * k2q + 2 * k3q + k4q);
        x.velocity += h / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v);
    }
    return x;
}

}  // namespace

TEST_CASE("plant factory registers the five WPILib-style models once") {
    sdk::PlantFactory f;
    REQUIRE(registerBuiltinPlants(f).ok());
    for (const char* key : {"dc_motor", "turret", "flywheel", "arm", "elevator"}) CHECK(f.contains(key));
    CHECK(registerBuiltinPlants(f).error().code == sdk::ErrorCode::AlreadyExists);
    sdk::PlantSpec unknown = neoSpec("nonsense");
    CHECK(f.createFor(unknown).error().code == sdk::ErrorCode::NotFound);
    CHECK(validatePlantSpec(unknown).error().code == sdk::ErrorCode::NotFound);
}

TEST_CASE("plant specs are validated field by field") {
    sdk::PlantSpec ok = neoSpec("flywheel");
    ok.momentOfInertiaKgM2 = 0.01;
    CHECK(validatePlantSpec(ok).ok());

    auto bad = [&](auto mutate, const char* field) {
        sdk::PlantSpec s = ok;
        mutate(s);
        const sdk::Status st = validatePlantSpec(s);
        REQUIRE_FALSE(st.ok());
        CHECK(st.error().message.find(field) != std::string::npos);
        // the factory refuses it too (returns null -> Internal)
        CHECK_FALSE(makeFactory().createFor(s).ok());
    };
    bad([](sdk::PlantSpec& s) { s.gearRatio = 0.0; }, "gearRatio");
    bad([](sdk::PlantSpec& s) { s.motorCount = 0; }, "motorCount");
    bad([](sdk::PlantSpec& s) { s.efficiency = 1.5; }, "efficiency");
    bad([](sdk::PlantSpec& s) { s.momentOfInertiaKgM2 = -1.0; }, "momentOfInertiaKgM2");
    bad([](sdk::PlantSpec& s) { s.viscousFrictionNmS = -1.0; }, "viscousFrictionNmS");
    bad([](sdk::PlantSpec& s) { s.maxPosition = -1.0; }, "minPosition/maxPosition");
    bad([](sdk::PlantSpec& s) { s.motor.stallCurrentA = 0.0; }, "motor");

    sdk::PlantSpec elevator = neoSpec("elevator");
    elevator.massKg = 0.0;
    CHECK(validatePlantSpec(elevator).error().message.find("massKg") != std::string::npos);
    elevator.massKg = 5.0;
    elevator.drumRadiusM = 0.0;
    CHECK(validatePlantSpec(elevator).error().message.find("drumRadiusM") != std::string::npos);
}

TEST_CASE("flywheel step response matches the analytic solution") {
    sdk::PlantSpec spec = neoSpec("flywheel");   // NEO x2, J = 0.00032, 1:1 like WPILib's LinearSystemIdTest
    spec.momentOfInertiaKgM2 = 0.00032;
    auto plant = makePlant(spec);

    const LinearSystem sys = dcMotorSystem(spec.motor, 2, 0.00032, 1.0);
    const double a = sys.A(1, 1), b = sys.B(1, 0), volts = 6.0;
    const double vInf = b * volts / -a;
    double t = 0.0;
    for (int i = 0; i < 200; ++i) {
        plant->step({volts, 0.0, kDt});
        t += kDt;
        const double v = vInf * (1.0 - std::exp(a * t));
        const double x = vInf * (t - (std::exp(a * t) - 1.0) / a);
        REQUIRE(plant->state().velocity == doctest::Approx(v).epsilon(1e-9));
        REQUIRE(plant->state().position == doctest::Approx(x).epsilon(1e-9));
    }
    // free speed of the gearbox at 6 V: V*Kv
    CHECK(plant->state().velocity == doctest::Approx(6.0 * spec.motor.kvRadPerSecPerVolt()).epsilon(1e-6));
    CHECK(plant->kind() == sdk::PlantKind::Angular);
    CHECK_FALSE(plant->atLowerLimit());
}

TEST_CASE("the exact discretization is independent of the step size") {
    sdk::PlantSpec spec = neoSpec("dc_motor", 1);
    spec.momentOfInertiaKgM2 = 0.01;
    spec.gearRatio = 5.0;
    auto coarse = makePlant(spec);
    auto fine = makePlant(spec);
    for (int i = 0; i < 10; ++i) coarse->step({8.0, 0.0, 0.02});
    for (int i = 0; i < 200; ++i) fine->step({8.0, 0.0, 0.001});
    CHECK(coarse->state().velocity == doctest::Approx(fine->state().velocity).epsilon(1e-9));
    CHECK(coarse->state().position == doctest::Approx(fine->state().position).epsilon(1e-9));
}

TEST_CASE("elevator with gravity follows the analytic affine solution and WPILib's gravity constant") {
    sdk::PlantSpec spec = neoSpec("elevator");   // NEO x2, 5 kg, 5 cm drum, 12:1 -> WPILib test constants
    spec.massKg = 5.0;
    spec.drumRadiusM = 0.05;
    spec.gearRatio = 12.0;
    spec.simulateGravity = true;
    auto plant = makePlant(spec);
    CHECK(plant->kind() == sdk::PlantKind::Linear);

    const LinearSystem sys = elevatorSystem(spec.motor, 2, 5.0, 0.05, 12.0);
    const double a = sys.A(1, 1), b = sys.B(1, 0), volts = 6.0;
    const double u = b * volts - kPlantGravityMps2;
    double t = 0.0;
    for (int i = 0; i < 100; ++i) {
        plant->step({volts, 0.0, kDt});
        t += kDt;
    }
    const double v = u / -a * (1.0 - std::exp(a * t));
    const double x = u / -a * (t - (std::exp(a * t) - 1.0) / a);
    CHECK(plant->state().velocity == doctest::Approx(v).epsilon(1e-9));
    CHECK(plant->state().position == doctest::Approx(x).epsilon(1e-9));
}

TEST_CASE("elevator holds position at the gravity feed-forward voltage") {
    sdk::PlantSpec spec = neoSpec("elevator");
    spec.massKg = 5.0;
    spec.drumRadiusM = 0.05;
    spec.gearRatio = 12.0;
    spec.simulateGravity = true;
    auto plant = makePlant(spec);
    const LinearSystem sys = elevatorSystem(spec.motor, 2, 5.0, 0.05, 12.0);
    const double kg = kPlantGravityMps2 / sys.B(1, 0);   // volts that cancel gravity at rest
    for (int i = 0; i < 400; ++i) plant->step({kg, 0.0, kDt});
    CHECK(plant->state().velocity == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(plant->state().position == doctest::Approx(0.0).epsilon(1e-9));
}

TEST_CASE("arm with gravity matches an independent high-resolution integration of WPILib's model") {
    // WPILib SingleJointedArmSim: J = 1/3 m L^2, alpha_g = -3/2 * 9.8 * cos(theta) / L
    const double length = 0.7, mass = 3.0, jKgM2 = mass * length * length / 3.0;
    sdk::PlantSpec spec = neoSpec("arm");
    spec.momentOfInertiaKgM2 = jKgM2;
    spec.massKg = mass;
    spec.armLengthM = length / 2.0;   // pivot to centre of mass
    spec.gearRatio = 50.0;
    spec.simulateGravity = true;
    auto plant = makePlant(spec);

    const LinearSystem sys = singleJointedArmSystem(spec.motor, 2, jKgM2, 50.0);
    const double a = sys.A(1, 1), b = sys.B(1, 0), volts = 2.5;
    sdk::PlantState ref{0.2, 0.0};
    plant->reset(ref);
    for (int i = 0; i < 400; ++i) plant->step({volts, 0.0, kDt});
    ref = integrate(ref, 400 * kDt, 1e-5, [&](double q, double v) {
        return a * v + b * volts - 1.5 * 9.8 * std::cos(q) / length;
    });
    CHECK(plant->state().position == doctest::Approx(ref.position).epsilon(1e-6));
    CHECK(plant->state().velocity == doctest::Approx(ref.velocity).epsilon(1e-6));
}

TEST_CASE("arm holds a horizontal and an inclined pose at the cos-scaled feed-forward voltage") {
    sdk::PlantSpec spec = neoSpec("arm");
    spec.momentOfInertiaKgM2 = 0.1;
    spec.massKg = 4.0;
    spec.armLengthM = 0.4;
    spec.gearRatio = 40.0;
    spec.simulateGravity = true;
    const LinearSystem sys = singleJointedArmSystem(spec.motor, 2, 0.1, 40.0);
    for (const double angle : {0.0, 0.6, -0.9}) {
        auto plant = makePlant(spec);
        plant->reset({angle, 0.0});
        const double hold = spec.massKg * kPlantGravityMps2 * spec.armLengthM * std::cos(angle) / 0.1 / sys.B(1, 0);
        for (int i = 0; i < 200; ++i) plant->step({hold, 0.0, kDt});
        CHECK(plant->state().position == doctest::Approx(angle).epsilon(1e-9));
        CHECK(std::abs(plant->state().velocity) < 1e-9);
    }
}

TEST_CASE("arm gravity respects the gravity angle offset") {
    sdk::PlantSpec spec = neoSpec("arm", 1);
    spec.momentOfInertiaKgM2 = 0.05;
    spec.massKg = 2.0;
    spec.armLengthM = 0.3;
    spec.gearRatio = 100.0;
    spec.simulateGravity = true;
    spec.gravityAngleOffsetRad = 1.0;
    auto plant = makePlant(spec);
    plant->reset({1.0, 0.0});   // gravity torque is maximal here: stronger pull than at the offset-free pose
    plant->step({0.0, 0.0, 0.01});
    sdk::PlantSpec noOffset = spec;
    noOffset.gravityAngleOffsetRad = 0.0;
    auto other = makePlant(noOffset);
    other->reset({1.0, 0.0});
    other->step({0.0, 0.0, 0.01});
    CHECK(plant->state().velocity < other->state().velocity);   // more negative acceleration
}

TEST_CASE("efficiency scales the dynamics rate like the analytic solution") {
    sdk::PlantSpec spec = neoSpec("flywheel", 1);
    spec.momentOfInertiaKgM2 = 0.004;
    spec.gearRatio = 2.0;
    spec.efficiency = 0.8;
    auto plant = makePlant(spec);
    const LinearSystem sys = dcMotorSystem(spec.motor, 1, 0.004, 2.0);
    const double a = 0.8 * sys.A(1, 1), b = 0.8 * sys.B(1, 0), volts = 9.0;
    for (int i = 0; i < 100; ++i) plant->step({volts, 0.0, kDt});
    const double t = 100 * kDt;
    CHECK(plant->state().velocity == doctest::Approx(b * volts / -a * (1.0 - std::exp(a * t))).epsilon(1e-6));
}

TEST_CASE("efficiency below one scales the holding voltage at standstill (motoring quadrant)") {
    sdk::PlantSpec spec = neoSpec("elevator");
    spec.massKg = 5.0;
    spec.drumRadiusM = 0.05;
    spec.gearRatio = 12.0;
    spec.simulateGravity = true;
    spec.efficiency = 0.5;
    auto plant = makePlant(spec);
    const LinearSystem sys = elevatorSystem(spec.motor, 2, 5.0, 0.05, 12.0);
    const double hold = kPlantGravityMps2 / (0.5 * sys.B(1, 0));
    for (int i = 0; i < 300; ++i) plant->step({hold, 0.0, kDt});
    CHECK(plant->state().position == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(std::abs(plant->state().velocity) < 1e-9);
}

TEST_CASE("viscous friction adds damping b/J to the velocity decay rate") {
    sdk::PlantSpec spec = neoSpec("flywheel", 1);
    spec.momentOfInertiaKgM2 = 0.01;
    spec.viscousFrictionNmS = 0.05;
    auto plant = makePlant(spec);
    const LinearSystem sys = dcMotorSystem(spec.motor, 1, 0.01, 1.0);
    const double a = sys.A(1, 1) - 0.05 / 0.01, b = sys.B(1, 0), volts = 5.0;
    for (int i = 0; i < 150; ++i) plant->step({volts, 0.0, kDt});
    const double t = 150 * kDt;
    CHECK(plant->state().velocity == doctest::Approx(b * volts / -a * (1.0 - std::exp(a * t))).epsilon(1e-9));
}

TEST_CASE("external load acts as a disturbance torque/force") {
    sdk::PlantSpec spec = neoSpec("elevator", 1);
    spec.massKg = 4.0;
    spec.drumRadiusM = 0.03;
    spec.gearRatio = 10.0;
    auto plant = makePlant(spec);
    const LinearSystem sys = elevatorSystem(spec.motor, 1, 4.0, 0.03, 10.0);
    const double a = sys.A(1, 1);
    const double load = -20.0;   // N
    for (int i = 0; i < 80; ++i) plant->step({0.0, load, kDt});
    const double t = 80 * kDt, u = load / 4.0;
    CHECK(plant->state().velocity == doctest::Approx(u / -a * (1.0 - std::exp(a * t))).epsilon(1e-9));
}

TEST_CASE("Coulomb friction: sticks below the threshold, slides above it, decelerates linearly when coasting") {
    sdk::PlantSpec spec = neoSpec("flywheel", 1);
    spec.momentOfInertiaKgM2 = 0.01;
    spec.coulombFrictionNm = 0.5;

    // Static torque = Kt * V / R * G. Pick voltages below / above 0.5 N*m.
    const double torquePerVolt = spec.motor.ktNmPerAmp() / spec.motor.resistanceOhm();
    {
        auto plant = makePlant(spec);
        for (int i = 0; i < 200; ++i) plant->step({0.8 * 0.5 / torquePerVolt, 0.0, kDt});
        CHECK(plant->state().position == 0.0);
        CHECK(plant->state().velocity == 0.0);
    }
    {
        auto plant = makePlant(spec);
        for (int i = 0; i < 200; ++i) plant->step({1.5 * 0.5 / torquePerVolt, 0.0, kDt});
        CHECK(plant->state().velocity > 1.0);
    }
    {
        // Cancel the back-EMF every step so only friction acts: v(t) = v0 - Fc/J * t.
        auto plant = makePlant(spec);
        plant->reset({0.0, 40.0});
        const double kv = spec.motor.kvRadPerSecPerVolt();
        for (int i = 0; i < 100; ++i) plant->step({plant->state().velocity / kv, 0.0, kDt});
        CHECK(plant->state().velocity == doctest::Approx(40.0 - 0.5 / 0.01 * 100 * kDt).epsilon(1e-9));
        // keeps decelerating to a standstill and stays there
        for (int i = 0; i < 400; ++i) plant->step({plant->state().velocity / kv, 0.0, kDt});
        CHECK(plant->state().velocity == 0.0);
    }
}

TEST_CASE("hard stops clamp the position, zero the velocity and release when driven away") {
    sdk::PlantSpec spec = neoSpec("elevator", 1);
    spec.massKg = 3.0;
    spec.drumRadiusM = 0.03;
    spec.gearRatio = 10.0;
    spec.minPosition = 0.0;
    spec.maxPosition = 0.5;
    auto plant = makePlant(spec);
    for (int i = 0; i < 2000 && !plant->atUpperLimit(); ++i) {
        plant->step({12.0, 0.0, kDt});
        REQUIRE(plant->state().position <= 0.5);
    }
    CHECK(plant->atUpperLimit());
    CHECK_FALSE(plant->atLowerLimit());
    CHECK(plant->state().position == 0.5);
    CHECK(plant->state().velocity == 0.0);
    // pressing into the stop keeps it there and draws stall current
    plant->step({12.0, 0.0, kDt});
    CHECK(plant->state().position == 0.5);
    CHECK(plant->motorCurrentA() == doctest::Approx(12.0 / spec.motor.resistanceOhm()).epsilon(1e-9));
    // reverse: leaves the stop and reaches the lower one
    plant->step({-12.0, 0.0, kDt});
    CHECK(plant->state().position < 0.5);
    CHECK_FALSE(plant->atUpperLimit());
    for (int i = 0; i < 4000 && !plant->atLowerLimit(); ++i) plant->step({-12.0, 0.0, kDt});
    CHECK(plant->atLowerLimit());
    CHECK(plant->state().position == 0.0);
    CHECK(plant->state().velocity == 0.0);
}

TEST_CASE("an arm falls under gravity onto the lower stop and a reset clamps into the stops") {
    sdk::PlantSpec spec = neoSpec("arm", 1);
    spec.momentOfInertiaKgM2 = 0.05;
    spec.massKg = 2.0;
    spec.armLengthM = 0.3;
    spec.gearRatio = 100.0;
    spec.simulateGravity = true;
    spec.minPosition = -0.2;
    spec.maxPosition = 1.2;
    auto plant = makePlant(spec);
    plant->reset({1.0, 0.0});
    // coast (back-EMF cancelled): free fall to the stop
    const double kv = spec.motor.kvRadPerSecPerVolt();
    for (int i = 0; i < 4000 && !plant->atLowerLimit(); ++i)
        plant->step({plant->motorShaftOmegaRadPerSec() / kv, 0.0, kDt});
    CHECK(plant->atLowerLimit());
    CHECK(plant->state().position == -0.2);
    plant->reset({5.0, 3.0});
    CHECK(plant->state().position == 1.2);
    CHECK(plant->state().velocity == 0.0);
    CHECK(plant->atUpperLimit());
}

TEST_CASE("flywheels ignore stops and integrate position for encoders, turrets honour them") {
    sdk::PlantSpec spec = neoSpec("flywheel", 1);
    spec.momentOfInertiaKgM2 = 0.001;
    spec.minPosition = -1.0;
    spec.maxPosition = 1.0;
    auto fly = makePlant(spec);
    for (int i = 0; i < 400; ++i) fly->step({12.0, 0.0, kDt});
    CHECK(fly->state().position > 100.0);
    CHECK_FALSE(fly->atUpperLimit());

    spec.model = "turret";
    auto turret = makePlant(spec);
    for (int i = 0; i < 400; ++i) turret->step({12.0, 0.0, kDt});
    CHECK(turret->state().position == 1.0);
    CHECK(turret->atUpperLimit());

    spec.minPosition = spec.maxPosition = 0.0;   // continuous turret
    auto continuous = makePlant(spec);
    for (int i = 0; i < 400; ++i) continuous->step({12.0, 0.0, kDt});
    CHECK(continuous->state().position > 100.0);
}

TEST_CASE("a linear dc_motor plant behaves like an elevator without gravity") {
    sdk::PlantSpec a = neoSpec("dc_motor", 1);
    a.kind = sdk::PlantKind::Linear;
    a.massKg = 2.0;
    a.drumRadiusM = 0.02;
    a.gearRatio = 6.0;
    sdk::PlantSpec b = a;
    b.model = "elevator";
    auto pa = makePlant(a), pb = makePlant(b);
    CHECK(pa->kind() == sdk::PlantKind::Linear);
    for (int i = 0; i < 50; ++i) {
        pa->step({7.0, 0.0, kDt});
        pb->step({7.0, 0.0, kDt});
    }
    CHECK(pa->state().position == pb->state().position);
    CHECK(pa->state().velocity == pb->state().velocity);
}

TEST_CASE("plant reports motor current and shaft speed consistently with the DC motor equations") {
    sdk::PlantSpec spec = neoSpec("dc_motor", 3);
    spec.momentOfInertiaKgM2 = 0.02;
    spec.gearRatio = 4.0;
    auto plant = makePlant(spec);
    plant->step({10.0, 0.0, kDt});   // from rest: current computed at the start state
    CHECK(plant->motorCurrentA() == doctest::Approx(3.0 * 10.0 / spec.motor.resistanceOhm()).epsilon(1e-9));
    plant->step({10.0, 0.0, kDt});
    CHECK(plant->motorShaftOmegaRadPerSec() == doctest::Approx(4.0 * plant->state().velocity));
    CHECK(plant->motorCurrentA() < 3.0 * 10.0 / spec.motor.resistanceOhm());   // back-EMF is building up
}

TEST_CASE("plant steps are deterministic and robust to bad input") {
    sdk::PlantSpec spec = neoSpec("arm", 1);
    spec.momentOfInertiaKgM2 = 0.05;
    spec.massKg = 2.0;
    spec.armLengthM = 0.3;
    spec.simulateGravity = true;
    auto a = makePlant(spec), b = makePlant(spec);
    for (int i = 0; i < 300; ++i) {
        const double v = std::sin(i * 0.05) * 6.0;
        a->step({v, 0.0, kDt});
        b->step({v, 0.0, kDt});
    }
    CHECK(a->state().position == b->state().position);
    CHECK(a->state().velocity == b->state().velocity);

    const sdk::PlantState before = a->state();
    a->step({5.0, 0.0, 0.0});                       // zero dt: no change
    a->step({5.0, 0.0, -1.0});                      // negative dt: no change
    CHECK(a->state().position == before.position);
    a->step({std::nan(""), std::nan(""), kDt});     // NaN inputs are treated as 0
    CHECK(std::isfinite(a->state().position));
    CHECK(std::isfinite(a->state().velocity));
}
