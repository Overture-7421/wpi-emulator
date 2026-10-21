#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

#include "wpiemu/core/mechanisms/linear_system.h"
#include "wpiemu/core/mechanisms/motor_catalog.h"
#include "wpiemu/sdk/core/units.h"

using namespace wpiemu::core::mechanisms;
namespace sdk = wpiemu::sdk;

namespace {
sdk::MotorSpec catalogMotor(const char* key) {
    const MotorCatalog c = MotorCatalog::builtin();
    const sdk::MotorSpec* s = c.spec(key);
    REQUIRE(s != nullptr);
    return *s;
}
}  // namespace

TEST_CASE("matrix exponential matches closed forms") {
    Matrix zero(3, 3);
    const Matrix e0 = expm(zero);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) CHECK(e0(i, j) == doctest::Approx(i == j ? 1.0 : 0.0));

    Matrix nilpotent(2, 2);
    nilpotent(0, 1) = 3.0;   // exp([[0,3],[0,0]]) = [[1,3],[0,1]]
    const Matrix en = expm(nilpotent);
    CHECK(en(0, 0) == doctest::Approx(1.0));
    CHECK(en(0, 1) == doctest::Approx(3.0));
    CHECK(en(1, 0) == doctest::Approx(0.0));

    Matrix scalar(1, 1);
    scalar(0, 0) = 7.5;   // needs several squarings
    CHECK(expm(scalar)(0, 0) == doctest::Approx(std::exp(7.5)).epsilon(1e-12));

    Matrix rot(2, 2);   // rotation generator: exp(t*J) is a rotation by t
    rot(0, 1) = -2.0;
    rot(1, 0) = 2.0;
    const Matrix r = expm(rot);
    CHECK(r(0, 0) == doctest::Approx(std::cos(2.0)).epsilon(1e-12));
    CHECK(r(1, 0) == doctest::Approx(std::sin(2.0)).epsilon(1e-12));
}

TEST_CASE("discretizeAB equals the analytic zero-order hold of a velocity-decay system") {
    // x' = a x + b u  =>  Ad = e^{a dt}, Bd = b (e^{a dt} - 1) / a
    Matrix a(1, 1), b(1, 1);
    a(0, 0) = -26.87032;
    b(0, 0) = 1354.166667;
    const double dt = 0.005;
    const DiscreteSystem d = discretizeAB(a, b, dt);
    CHECK(d.Ad(0, 0) == doctest::Approx(std::exp(a(0, 0) * dt)).epsilon(1e-13));
    CHECK(d.Bd(0, 0) == doctest::Approx(b(0, 0) * (std::exp(a(0, 0) * dt) - 1.0) / a(0, 0)).epsilon(1e-12));
}

TEST_CASE("LinearSystemId equivalents reproduce WPILib's known values") {
    // wpimath LinearSystemIdTest: DCMotor.getNEO(2)
    const sdk::MotorSpec neo = catalogMotor("neo");

    const LinearSystem elevator = elevatorSystem(neo, 2, 5.0, 0.05, 12.0);
    CHECK(elevator.A(0, 1) == 1.0);
    CHECK(elevator.A(1, 1) == doctest::Approx(-99.05473).epsilon(1e-7));
    CHECK(elevator.B(1, 0) == doctest::Approx(20.8).epsilon(1e-7));

    const LinearSystem flywheel = flywheelSystem(neo, 2, 0.00032, 1.0);
    CHECK(flywheel.A(0, 0) == doctest::Approx(-26.87032).epsilon(1e-6));
    CHECK(flywheel.B(0, 0) == doctest::Approx(1354.166667).epsilon(1e-6));

    const LinearSystem dc = dcMotorSystem(neo, 2, 0.00032, 1.0);
    CHECK(dc.A(1, 1) == doctest::Approx(-26.87032).epsilon(1e-6));
    CHECK(dc.B(1, 0) == doctest::Approx(1354.166667).epsilon(1e-6));

    const LinearSystem arm = singleJointedArmSystem(neo, 2, 0.00032, 1.0);
    CHECK(arm.A(1, 1) == dc.A(1, 1));
    CHECK(arm.B(1, 0) == dc.B(1, 0));
}

TEST_CASE("LinearSystemId equivalents reject non-physical input with NaN") {
    const sdk::MotorSpec neo = catalogMotor("neo");
    CHECK(std::isnan(elevatorSystem(neo, 1, 0.0, 0.05, 1.0).A(1, 1)));
    CHECK(std::isnan(elevatorSystem(neo, 1, 5.0, 0.05, 0.0).B(1, 0)));
    CHECK(std::isnan(flywheelSystem(neo, 0, 0.001, 1.0).A(0, 0)));
    CHECK(std::isnan(dcMotorSystem(neo, 1, -1.0, 1.0).A(1, 1)));
}

TEST_CASE("the motor count scales Kt/R like WPILib's DCMotor aggregation") {
    const sdk::MotorSpec neo = catalogMotor("neo");
    const LinearSystem one = dcMotorSystem(neo, 1, 0.01, 1.0);
    const LinearSystem four = dcMotorSystem(neo, 4, 0.01, 1.0);
    CHECK(four.B(1, 0) == doctest::Approx(4.0 * one.B(1, 0)).epsilon(1e-12));
    CHECK(four.A(1, 1) == doctest::Approx(4.0 * one.A(1, 1)).epsilon(1e-12));
}

TEST_CASE("every gearbox system equals WPILib's n-motor DCMotor equations for every catalog motor") {
    double worst = 0.0;
    const auto compare = [&](double actual, double expected) {
        worst = std::max(worst, std::abs(actual - expected) / std::abs(expected));
    };
    const MotorCatalog catalog = MotorCatalog::builtin();
    for (const MotorCatalogEntry& entry : catalog.entries()) {
        const sdk::MotorSpec& m = entry.spec;
        for (const int n : {1, 2, 3, 4, 8}) {
            // DCMotor(V, tau*n, I*n, Ifree*n, w): R = V / I, Kt = tau / I, Kv = w / (V - R*Ifree)
            const double stall = m.stallCurrentA * n;
            const double r = m.nominalVoltageV / stall;
            const double kt = m.stallTorqueNm * n / stall;
            const double kv = m.freeSpeedRadPerSec / (m.nominalVoltageV - r * m.freeCurrentA * n);
            for (const double gearing : {0.5, 1.0, 7.5, 100.0})
                for (const double inertiaOrMass : {1e-4, 0.01, 3.0}) {
                    const double j = inertiaOrMass;
                    const LinearSystem flywheel = flywheelSystem(m, n, j, gearing);
                    compare(flywheel.A(0, 0), -gearing * gearing * kt / (kv * r * j));
                    compare(flywheel.B(0, 0), gearing * kt / (r * j));
                    const LinearSystem dc = dcMotorSystem(m, n, j, gearing);
                    CHECK(dc.A(0, 1) == 1.0);
                    compare(dc.A(1, 1), -gearing * gearing * kt / (kv * r * j));
                    compare(dc.B(1, 0), gearing * kt / (r * j));
                    for (const double radius : {0.01, 0.05}) {
                        const LinearSystem elevator = elevatorSystem(m, n, j, radius, gearing);
                        compare(elevator.A(1, 1), -gearing * gearing * kt / (r * radius * radius * j * kv));
                        compare(elevator.B(1, 0), gearing * kt / (r * radius * j));
                    }
                }
        }
    }
    CHECK(worst < 1e-12);
}

TEST_CASE("motor catalog carries the WPILib DCMotor table") {
    const MotorCatalog c = MotorCatalog::builtin();
    struct Row {
        const char* key;
        double torque, stall, free, rpm;
    };
    // allwpilib 2026 DCMotor.java
    const Row rows[] = {
        {"kraken_x60", 7.09, 366, 2, 6000},      {"kraken_x60_foc", 9.37, 483, 2, 5800},
        {"kraken_x44", 4.11, 279, 2, 7758},      {"kraken_x44_foc", 5.01, 329, 2, 7368},
        {"falcon_500", 4.69, 257, 1.5, 6380},    {"falcon_500_foc", 5.84, 304, 1.5, 6080},
        {"minion", 3.17, 211, 2, 7704},          {"neo", 2.6, 105, 1.8, 5676},
        {"neo_550", 0.97, 100, 1.4, 11000},      {"neo_vortex", 3.60, 211, 3.6, 6784},
        {"cim", 2.42, 133, 2.7, 5310},           {"mini_cim", 1.41, 89, 3, 5840},
        {"bag", 0.43, 53, 1.8, 13180},           {"vex_775pro", 0.71, 134, 0.7, 18730},
        {"andymark_rs775_125", 0.28, 18, 1.6, 5800}, {"banebots_rs775", 0.72, 97, 2.7, 13050},
        {"andymark_9015", 0.36, 71, 3.7, 14270}, {"banebots_rs550", 0.38, 84, 0.4, 19000},
    };
    for (const Row& r : rows) {
        INFO(r.key);
        const sdk::MotorSpec* s = c.spec(r.key);
        REQUIRE(s != nullptr);
        CHECK(s->nominalVoltageV == 12.0);
        CHECK(s->stallTorqueNm == r.torque);
        CHECK(s->stallCurrentA == r.stall);
        CHECK(s->freeCurrentA == r.free);
        CHECK(s->freeSpeedRadPerSec == doctest::Approx(sdk::rpmToRadPerSec(r.rpm)));
        CHECK(isValidMotorSpec(*s));
    }
    const sdk::MotorSpec* romi = c.spec("romi_built_in");
    REQUIRE(romi != nullptr);
    CHECK(romi->nominalVoltageV == 4.5);
    CHECK(c.keys().size() == c.entries().size());
    CHECK(c.entries().size() == sdk::motors::presets().size());   // the catalog accepted every SDK row
    CHECK(c.spec("nope") == nullptr);
}

TEST_CASE("every catalog motor reproduces its stall and free operating points") {
    const MotorCatalog c = MotorCatalog::builtin();
    for (const MotorCatalogEntry& e : c.entries()) {
        INFO(e.key);
        auto model = c.createModel(e.key);
        REQUIRE(model.ok());
        const sdk::IMotorModel& m = *model.value();
        const double v = e.spec.nominalVoltageV;
        CHECK(m.currentA(v, 0.0) == doctest::Approx(e.spec.stallCurrentA).epsilon(1e-9));
        CHECK(m.torqueNm(v, 0.0) == doctest::Approx(e.spec.stallTorqueNm).epsilon(1e-9));
        CHECK(m.currentA(v, e.spec.freeSpeedRadPerSec) == doctest::Approx(e.spec.freeCurrentA).epsilon(1e-9));
    }
}

TEST_CASE("catalog add rejects duplicates, empty keys and non-physical motors") {
    MotorCatalog c = MotorCatalog::builtin();
    sdk::MotorSpec custom = catalogMotor("neo");
    custom.name = "custom";
    CHECK(c.add({"custom", "Custom", custom}).ok());
    CHECK(c.add({"custom", "Custom", custom}).error().code == sdk::ErrorCode::AlreadyExists);
    CHECK(c.add({"", "x", custom}).error().code == sdk::ErrorCode::InvalidArgument);
    sdk::MotorSpec bad = custom;
    bad.stallCurrentA = 0.0;
    CHECK(c.add({"bad", "Bad", bad}).error().code == sdk::ErrorCode::InvalidArgument);
    bad = custom;
    bad.freeCurrentA = 1000.0;   // free current above stall current
    CHECK_FALSE(isValidMotorSpec(bad));
    CHECK(c.createModel("missing").error().code == sdk::ErrorCode::NotFound);
}

TEST_CASE("current limited motor model clamps current and torque, and reports the limited voltage") {
    const MotorCatalog c = MotorCatalog::builtin();
    const sdk::MotorSpec neo = *c.spec("neo");
    CurrentLimitedMotorModel limited(std::make_unique<sdk::DcMotorModel>(neo), 40.0);

    CHECK(limited.currentA(12.0, 0.0) == doctest::Approx(40.0));          // stall: 105 A -> 40 A
    CHECK(limited.currentA(-12.0, 0.0) == doctest::Approx(-40.0));
    CHECK(limited.torqueNm(12.0, 0.0) == doctest::Approx(40.0 * neo.ktNmPerAmp()));
    CHECK(limited.currentA(2.0, 0.0) == doctest::Approx(2.0 / neo.resistanceOhm()));   // below the limit

    // limited voltage reproduces exactly the limited current in the unlimited model
    const double omega = 100.0;
    const double vLim = limited.limitedVoltage(12.0, omega);
    sdk::DcMotorModel ideal(neo);
    CHECK(ideal.currentA(vLim, omega) == doctest::Approx(limited.currentA(12.0, omega)).epsilon(1e-9));
    CHECK(vLim < 12.0);

    CurrentLimitedMotorModel unlimited(std::make_unique<sdk::DcMotorModel>(neo), 0.0);
    CHECK(unlimited.currentA(12.0, 0.0) == doctest::Approx(neo.stallCurrentA));
    CHECK(unlimited.limitedVoltage(7.0, 3.0) == 7.0);
}
