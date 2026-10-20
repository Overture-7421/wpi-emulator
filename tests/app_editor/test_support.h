#pragma once

// Shared helpers for the editor tests.

#include <doctest/doctest.h>

#include "editor_backend.h"
#include "fake_editor_backend.h"
#include "model/config_ops.h"
#include "model/selection.h"
#include "viewmodel/editor_controller.h"
#include "viewmodel/viewport_controller.h"

namespace wpiemu::app::editor::testing {

void pumpEvents(int milliseconds);

inline EditorCatalog catalog() { return FakeEditorBackend::sampleCatalog(); }
inline sdk::RobotConfig robot() { return FakeEditorBackend::sampleConfig(); }

inline bool approx(float a, float b, float eps = 1e-4f) { return a > b - eps && a < b + eps; }
inline bool approx(const sdk::Vec3& a, const sdk::Vec3& b, float eps = 1e-4f) {
    return approx(a.x, b.x, eps) && approx(a.y, b.y, eps) && approx(a.z, b.z, eps);
}

// World position of a mechanism root (display frame).
inline sdk::Vec3 rootWorld(const sdk::RobotConfig& c, const EditorCatalog& cat, const std::string& id) {
    return ops::mechanismRootWorld(c, cat, id).position;
}

// Fake backend + editor + viewport (800x600 view, snapping off); screen() projects a world point.
struct ViewFixture {
    FakeEditorBackend backend;
    EditorController editor{backend};
    ViewportController view{editor};
    ViewFixture() {
        editor.setAutosaveDelayMs(0);
        view.setViewSize(800, 600);
        editor.settings().setSnapEnabled(false);
    }
    QPointF screen(const sdk::Vec3& world) {
        const auto px = view.camera().project(world, 800.f, 600.f);
        REQUIRE(px);
        return {static_cast<qreal>(px->first), static_cast<qreal>(px->second)};
    }
};

}  // namespace wpiemu::app::editor::testing
