#include <doctest/doctest.h>

#include <memory>
#include <vector>

#include "wpiemu/sdk/dynamics/plant.h"
#include "wpiemu/sdk/physics/drivetrain.h"
#include "wpiemu/sdk/physics/tire_model.h"
#include "wpiemu/sdk/testing/fake_physics_world.h"

using namespace wpiemu::sdk;
using wpiemu::sdk::testing::FakePhysicsWorld;

namespace {

BodyId makeDynamicBody(IPhysicsWorld& w, Vec3 at = {0.f, 0.f, 1.f}) {
    BodyDesc d;
    d.type = BodyType::Dynamic;
    d.pose.position = at;
    const BodyId b = w.createBody(d);
    MassProperties mp;
    mp.massKg = 10.f;
    w.setMassProperties(b, mp);
    return b;
}

}  // namespace

TEST_CASE("fake physics world integrates gravity and rests on the floor") {
    FakePhysicsWorld w;
    const BodyId b = makeDynamicBody(w);
    for (int i = 0; i < 400; ++i) w.step(0.005f);
    CHECK(w.pose(b).position.z == doctest::Approx(0.f).epsilon(1e-4));
    CHECK(w.stats().bodies == 1);
}

TEST_CASE("fake physics world: forces last one step, stateHash is deterministic") {
    auto run = [] {
        FakePhysicsWorld w;
        w.setFloorEnabled(false);
        const BodyId b = makeDynamicBody(w, {});
        w.applyForceToCenter(b, {10.f, 0.f, 0.f});
        w.step(0.1f);
        const float vx = w.linearVelocity(b).x;
        w.step(0.1f);  // no force re-applied: velocity unchanged in x
        CHECK(w.linearVelocity(b).x == doctest::Approx(vx));
        return w.stateHash();
    };
    CHECK(run() == run());
}

TEST_CASE("fake physics world: off-centre force creates torque") {
    FakePhysicsWorld w;
    w.setFloorEnabled(false);
    const BodyId b = makeDynamicBody(w, {});
    w.applyForce(b, {0.f, 5.f, 0.f}, {1.f, 0.f, 0.f});  // +Y force at +X => +Z torque
    w.step(0.1f);
    CHECK(w.angularVelocity(b).z > 0.f);
}

TEST_CASE("fake physics world: sensor begin/end events with tags and filters") {
    FakePhysicsWorld w;
    w.setFloorEnabled(false);

    const BodyId zoneBody = w.createBody(BodyDesc{});
    ShapeDesc zone;
    zone.geometry = BoxShape{{1.f, 1.f, 1.f}};
    zone.sensor = true;
    zone.sensorEvents = true;
    zone.category = layer::Sensor;
    zone.mask = layer::Fuel;
    zone.userTag = 111;
    const ShapeId sensor = w.addShape(zoneBody, zone);

    BodyDesc ball;
    ball.type = BodyType::Dynamic;
    ball.pose.position = {5.f, 0.f, 0.f};
    const BodyId ballBody = w.createBody(ball);
    ShapeDesc ballShape;
    ballShape.geometry = SphereShape{0.1f};
    ballShape.category = layer::Fuel;
    ballShape.sensorEvents = true;
    ballShape.userTag = 222;
    const ShapeId visitor = w.addShape(ballBody, ballShape);

    w.step(0.005f);
    CHECK(w.sensorEvents().empty());

    w.setPose(ballBody, Pose3{{0.f, 0.f, 0.f}, {}});
    w.step(0.005f);
    REQUIRE(w.sensorEvents().size() == 1);
    CHECK(w.sensorEvents()[0].begin);
    CHECK(w.sensorEvents()[0].sensorTag == 111);
    CHECK(w.sensorEvents()[0].visitorTag == 222);
    std::vector<ShapeId> inside;
    w.shapesOverlapping(sensor, inside);
    CHECK(inside == std::vector<ShapeId>{visitor});

    w.step(0.005f);
    CHECK(w.sensorEvents().empty());  // events are per-step transients

    w.destroyBody(ballBody);          // absorbing a piece: destroy outside event iteration
    w.step(0.005f);
    REQUIRE(w.sensorEvents().size() == 1);
    CHECK_FALSE(w.sensorEvents()[0].begin);
    CHECK_FALSE(w.isValid(visitor));
}

TEST_CASE("fake physics world: ray cast hits the floor on the Field layer only") {
    FakePhysicsWorld w;
    const RayHit hit = w.rayCastClosest({1.f, 2.f, 1.f}, {0.f, 0.f, -1.f}, 2.f, layer::Field);
    CHECK(hit.hit);
    CHECK(hit.point.z == doctest::Approx(0.f));
    CHECK(hit.normal.z == 1.f);
    CHECK_FALSE(w.rayCastClosest({1.f, 2.f, 1.f}, {0.f, 0.f, -1.f}, 0.5f, layer::Field).hit);
    CHECK_FALSE(w.rayCastClosest({1.f, 2.f, 1.f}, {0.f, 0.f, -1.f}, 2.f, layer::Fuel).hit);
}

TEST_CASE("invalid physics ids are tolerated with neutral results") {
    FakePhysicsWorld w;
    CHECK(w.pose(BodyId{}).position == Vec3{});
    w.applyForceToCenter(BodyId{999}, {1.f, 0.f, 0.f});
    CHECK_FALSE(w.isValid(ShapeId{5}));
}

namespace {

class StubTire final : public ITireModel {
public:
    std::string_view id() const override { return "stub"; }
    TireOutput solve(const TireParams& p, const TireInput& in, const WheelDrive& d) const override {
        TireOutput o;
        o.fLongN = p.mu * in.normalLoadN;
        o.omegaNext = d.omegaRadPerSec;
        return o;
    }
};

class StubDrivetrain final : public IDrivetrain {
public:
    explicit StubDrivetrain(const DrivetrainBuildArgs& a) : spec_(a.spec), world_(a.world), body_(a.chassisBody), tire_(a.tire) {
        states_.resize(spec_.wheels.size());
    }
    std::string_view kind() const override { return spec_.kind; }
    const DrivetrainSpec& spec() const override { return spec_; }
    std::vector<SlotSpec> slotSpecs() const override {
        std::vector<SlotSpec> out;
        for (const auto& w : spec_.wheels) out.push_back({w.name + ".drive", w.name + " drive", SlotType::Motor, true, ""});
        return out;
    }
    void step(std::span<const WheelCommand> cmds, float dt) override {
        for (std::size_t i = 0; i < cmds.size(); ++i) {
            const TireOutput o = tire_.solve(spec_.wheels[i].tire, TireInput{100.f, 0.f, 0.f, 10.f, 0.05f, dt}, WheelDrive{});
            world_.applyForce(body_, {o.fLongN, 0.f, 0.f}, spec_.wheels[i].positionM);
            states_[i].force = {o.fLongN, 0.f, 0.f};
        }
    }
    std::span<const WheelState> wheelStates() const override { return states_; }
    void reset() override {}

private:
    DrivetrainSpec spec_;
    IPhysicsWorld& world_;
    BodyId body_;
    const ITireModel& tire_;
    std::vector<WheelState> states_;
};

}  // namespace

TEST_CASE("DrivetrainRegistry builds an IDrivetrain from injected dependencies") {
    DrivetrainRegistry registry;
    CHECK(registry.registerDrivetrain(drivetrain_kind::kTank, {"Tank", ""}, [](const DrivetrainBuildArgs& a) {
                     return std::make_unique<StubDrivetrain>(a);
                 }).ok());

    FakePhysicsWorld world;
    const BodyId chassis = makeDynamicBody(world);
    const StubTire tire;
    DrivetrainSpec spec;
    spec.kind = drivetrain_kind::kTank;
    spec.wheels.resize(2);
    spec.wheels[0].name = "left";
    spec.wheels[1].name = "right";
    const ChassisSpec chassisSpec;

    auto dt = registry.create(drivetrain_kind::kTank, DrivetrainBuildArgs{spec, chassisSpec, world, chassis, tire});
    REQUIRE(dt.ok());
    CHECK(dt.value()->slotSpecs().size() == 2);

    const WheelCommand cmds[2] = {{6.0, 0.0}, {6.0, 0.0}};
    dt.value()->step(cmds, 0.005f);
    CHECK(world.pendingForces().size() == 2);
    CHECK(dt.value()->wheelStates()[0].force.x == doctest::Approx(110.f));  // mu 1.1 * 100 N
}

TEST_CASE("PlantFactory creates plants by model key") {
    class ConstPlant final : public IPlant {
    public:
        explicit ConstPlant(const PlantSpec& s) : spec_(s) {}
        PlantKind kind() const override { return spec_.kind; }
        const PlantSpec& spec() const override { return spec_; }
        void reset(const PlantState& s) override { state_ = s; }
        PlantState state() const override { return state_; }
        PlantState step(const PlantInput& in) override {
            state_.velocity += in.voltageV * in.dtSeconds;
            state_.position += state_.velocity * in.dtSeconds;
            return state_;
        }
        double motorCurrentA() const override { return 0.0; }
        double motorShaftOmegaRadPerSec() const override { return state_.velocity * spec_.gearRatio; }
        bool atLowerLimit() const override { return false; }
        bool atUpperLimit() const override { return false; }

    private:
        PlantSpec spec_;
        PlantState state_;
    };

    PlantFactory factory;
    CHECK(factory.registerPlant("const", [](const PlantSpec& s) { return std::make_unique<ConstPlant>(s); }).ok());
    PlantSpec spec;
    spec.model = "const";
    auto plant = factory.createFor(spec);
    REQUIRE(plant.ok());
    plant.value()->step(PlantInput{2.0, 0.0, 0.5});
    CHECK(plant.value()->state().velocity == doctest::Approx(1.0));
    spec.model = "missing";
    CHECK(factory.createFor(spec).error().code == ErrorCode::NotFound);
}
