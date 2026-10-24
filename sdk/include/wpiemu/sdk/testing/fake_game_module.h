#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/config/plugin_config.h"
#include "wpiemu/sdk/game/game_module.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::sdk::testing {

class FakeMechanism final : public IMechanism {
public:
    explicit FakeMechanism(const MechanismBuildContext& context)
        : id_(context.config.id), type_(context.config.type) {}

    const std::string& id() const override { return id_; }
    std::string_view type() const override { return type_; }
    void tick(SimContext&, double) override { ++ticks_; }
    void reset() override { ticks_ = 0; }
    MechanismState state() const override {
        MechanismState result;
        result.id = id_;
        result.type = type_;
        result.values.push_back({"ticks", static_cast<double>(ticks_), ""});
        return result;
    }

private:
    std::string id_;
    std::string type_;
    int ticks_ = 0;
};

class FakeGameModule : public IGameModule {
public:
    struct Counters {
        int gameStateSteps = 0;
        int onTickCalls = 0;
        int registerMechanismsCalls = 0;
        int createFieldCalls = 0;
        int createGameStateCalls = 0;
        double level = 0.0;  // published as the game value "<id>.level"
    };

    FakeGameModule() : FakeGameModule("game_fake", 2999) {}
    FakeGameModule(std::string id, int year) : id_(std::move(id)), year_(year), counters_(std::make_shared<Counters>()) {}

    GameMetadata metadata() const override {
        GameMetadata result;
        result.id = id_;
        result.displayName = "Fake game";
        result.year = year_;
        return result;
    }

    Status registerMechanisms(MechanismFactory& factory) override {
        ++counters_->registerMechanismsCalls;
        MechanismTemplateInfo info;
        info.type = id_ + ".fake_mechanism";
        info.displayName = "Fake mechanism";
        info.origin = id_;
        info.slotSpecs.push_back({"motor", "Motor", SlotType::Motor, true, "", "", 0});
        PropertySpec gearRatio;
        gearRatio.key = "gearRatio";
        gearRatio.label = "Gear ratio";
        gearRatio.type = PropertyType::Double;
        gearRatio.defaultValue = 1.0;
        gearRatio.min = 0.01;
        gearRatio.max = 1000.0;
        info.properties.properties.push_back(std::move(gearRatio));
        return factory.registerTemplate(std::move(info), [](const MechanismBuildContext& context) {
            return std::make_unique<FakeMechanism>(context);
        });
    }

    Result<std::unique_ptr<IField>> createField(IPhysicsWorld&) override {
        ++counters_->createFieldCalls;
        class FakeField final : public IField {
        public:
            FakeField() {
                description_.name = "Fake field";
                description_.lengthM = 16.f;
                description_.widthM = 8.f;
            }
            const FieldDescription& description() const override { return description_; }

        private:
            FieldDescription description_;
        };
        return std::unique_ptr<IField>(std::make_unique<FakeField>());
    }

    Result<std::unique_ptr<IGameState>> createGameState(SimContext&) override {
        ++counters_->createGameStateCalls;
        class FakeState final : public IGameState {
        public:
            FakeState(std::string pluginId, std::shared_ptr<Counters> counters)
                : pluginId_(std::move(pluginId)), counters_(std::move(counters)) {}
            std::string_view pluginId() const override { return pluginId_; }
            void reset(SimContext&) override { steps_ = 0; }
            void step(SimContext&, double) override {
                ++steps_;
                ++counters_->gameStateSteps;
            }
            void publish(SnapshotBuilder& out) const override {
                out.setHud(pluginId_ + ".steps", steps_);
            }
            std::optional<double> gameValue(std::string_view id) const override {
                if (id == pluginId_ + ".level") return counters_->level;
                if (id == pluginId_ + ".steps") return static_cast<double>(steps_);
                return std::nullopt;
            }

        private:
            std::string pluginId_;
            std::shared_ptr<Counters> counters_;
            int steps_ = 0;
        };
        return std::unique_ptr<IGameState>(std::make_unique<FakeState>(id_, counters_));
    }

    PluginConfigSchema contributeConfigSchema() const override {
        PluginConfigSchema schema;
        schema.schemaVersion = 1;
        PropertySpec capacity;
        capacity.key = "capacity";
        capacity.label = "Capacity";
        capacity.type = PropertyType::Int;
        capacity.defaultValue = 3;
        capacity.min = 1;
        capacity.max = 100;
        schema.properties.properties.push_back(std::move(capacity));
        return schema;
    }

    std::vector<ZoneKindInfo> contributeZoneKinds() const override {
        ZoneKindInfo kind;
        kind.kind = id_ + ".fake_zone";
        kind.displayName = "Fake zone";
        PropertySpec capacity;
        capacity.key = "capacity";
        capacity.label = "Capacity";
        capacity.type = PropertyType::Int;
        capacity.defaultValue = 3;
        capacity.min = 1;
        capacity.max = 100;
        kind.properties.properties.push_back(std::move(capacity));
        return {kind};
    }

    std::vector<GameValueInfo> contributeGameValues() const override {
        return {{id_ + ".level", "Level", "Test level", PropertyUnit::Count},
                {id_ + ".steps", "Steps", "Game state steps", PropertyUnit::Count}};
    }

    FieldFrame fieldFrame() const override { return {16.f, 8.f, FieldSymmetry::Rotational}; }

    std::vector<EditorPanelDescriptor> contributeEditorPanels() const override {
        EditorPanelDescriptor panel;
        panel.id = id_ + ".panel";
        panel.title = "Fake";
        return {panel};
    }

    std::vector<HudWidgetDescriptor> contributeHud() const override {
        HudWidgetDescriptor widget;
        widget.id = id_ + ".steps";
        widget.title = "Steps";
        widget.valueKey = id_ + ".steps";
        return {widget};
    }

    void onTick(SimContext&, double) override { ++counters_->onTickCalls; }
    std::shared_ptr<Counters> counters() const { return counters_; }

private:
    std::string id_;
    int year_;
    std::shared_ptr<Counters> counters_;
};

}  // namespace wpiemu::sdk::testing
