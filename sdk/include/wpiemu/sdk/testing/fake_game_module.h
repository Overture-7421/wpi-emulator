#pragma once

// A configurable fake year plugin + fake mechanism, for testing registries, the core runtime's
// plugin handling and UI descriptor rendering without any real game code.

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/sim_context.h"
#include "wpiemu/sdk/game/game_module.h"
#include "wpiemu/sdk/game/null_objects.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::sdk::testing {

class FakeMechanism final : public IMechanism {
public:
    explicit FakeMechanism(const MechanismBuildContext& ctx) : id_(ctx.config.id), type_(ctx.config.type) {}
    const std::string& id() const override { return id_; }
    std::string_view type() const override { return type_; }
    void tick(SimContext&, double dt) override { ++ticks; time += dt; }
    void reset() override { ticks = 0; time = 0.0; }
    MechanismState state() const override {
        MechanismState s;
        s.id = id_;
        s.type = type_;
        s.active = ticks > 0;
        s.values.push_back({"ticks", static_cast<double>(ticks), ""});
        return s;
    }
    int ticks = 0;
    double time = 0.0;

private:
    std::string id_;
    std::string type_;
};

class FakeGameState final : public IGameState {
public:
    FakeGameState(std::string id, int* stepCounter) : id_(std::move(id)), steps_(stepCounter) {}
    std::string_view pluginId() const override { return id_; }
    void reset(SimContext&) override { local_ = 0; }
    void step(SimContext&, double) override { ++local_; if (steps_ != nullptr) ++*steps_; }
    void publish(SnapshotBuilder& out) const override { out.setHud(id_ + ".steps", local_); }

private:
    std::string id_;
    int* steps_;
    int local_ = 0;
};

class FakeGameModule : public IGameModule {
public:
    struct Counters {
        int registerMechanismsCalls = 0;
        int createFieldCalls = 0;
        int createGameStateCalls = 0;
        int onTickCalls = 0;
        int gameStateSteps = 0;
    };

    explicit FakeGameModule(std::string id = "fake_game", int year = 2999)
        : id_(std::move(id)), year_(year), counters_(std::make_shared<Counters>()) {}

    // Observe calls even after ownership moved into a registry.
    std::shared_ptr<Counters> counters() const { return counters_; }

    GameMetadata metadata() const override {
        GameMetadata m;
        m.id = id_;
        m.displayName = "Fake " + id_;
        m.year = year_;
        return m;
    }
    Status registerMechanisms(MechanismFactory& factory) override {
        ++counters_->registerMechanismsCalls;
        MechanismTemplateInfo info;
        info.type = id_ + ".fake_mechanism";
        info.displayName = "Fake mechanism";
        info.origin = id_;
        info.slotSpecs.push_back({"motor", "Motor", SlotType::Motor, true, ""});
        return factory.registerTemplate(std::move(info), [](const MechanismBuildContext& ctx) {
            return std::make_unique<FakeMechanism>(ctx);
        });
    }
    Result<std::unique_ptr<IField>> createField(IPhysicsWorld&) override {
        ++counters_->createFieldCalls;
        FieldDescription d;
        d.name = id_;
        d.lengthM = 16.f;
        d.widthM = 8.f;
        return std::unique_ptr<IField>(std::make_unique<EmptyField>(std::move(d)));
    }
    Result<std::unique_ptr<IGameState>> createGameState(SimContext&) override {
        ++counters_->createGameStateCalls;
        return std::unique_ptr<IGameState>(std::make_unique<FakeGameState>(id_, &counters_->gameStateSteps));
    }
    PluginConfigSchema contributeConfigSchema() const override {
        PluginConfigSchema s;
        s.schemaVersion = 1;
        PropertySpec p;
        p.key = "capacity";
        p.label = "Capacity";
        p.type = PropertyType::Int;
        p.defaultValue = 3;
        p.min = 0;
        p.max = 100;
        s.properties.properties.push_back(std::move(p));
        return s;
    }
    std::vector<EditorPanelDescriptor> contributeEditorPanels() const override {
        EditorPanelDescriptor p;
        p.id = id_ + ".panel";
        p.title = "Fake panel";
        p.properties = contributeConfigSchema().properties;
        return {p};
    }
    std::vector<HudWidgetDescriptor> contributeHud() const override {
        HudWidgetDescriptor w;
        w.id = id_ + ".steps";
        w.title = "Steps";
        w.valueKey = id_ + ".steps";
        return {w};
    }
    // One Volume kind "<id>.fake_zone" with an Int property "capacity" in [0, 100] (default 3).
    std::vector<ZoneKindInfo> contributeZoneKinds() const override {
        ZoneKindInfo k;
        k.kind = id_ + ".fake_zone";
        k.displayName = "Fake zone";
        k.properties = contributeConfigSchema().properties;
        return {std::move(k)};
    }
    void onTick(SimContext&, double) override { ++counters_->onTickCalls; }

private:
    std::string id_;
    int year_;
    std::shared_ptr<Counters> counters_;
};

}  // namespace wpiemu::sdk::testing
