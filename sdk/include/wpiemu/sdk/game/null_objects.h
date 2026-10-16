#pragma once

// Do-nothing implementations for plugin stubs and tests.

#include <string>
#include <string_view>
#include <utility>

#include "wpiemu/sdk/game/field.h"
#include "wpiemu/sdk/game/game_state.h"

namespace wpiemu::sdk {

class EmptyField final : public IField {
public:
    explicit EmptyField(FieldDescription description = {}) : description_(std::move(description)) {}
    const FieldDescription& description() const override { return description_; }

private:
    FieldDescription description_;
};

class NullGameState final : public IGameState {
public:
    explicit NullGameState(std::string pluginId = "null") : pluginId_(std::move(pluginId)) {}
    std::string_view pluginId() const override { return pluginId_; }
    void reset(SimContext&) override {}
    void step(SimContext&, double) override {}
    void publish(SnapshotBuilder&) const override {}

private:
    std::string pluginId_;
};

}  // namespace wpiemu::sdk
