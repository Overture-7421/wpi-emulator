#pragma once

// Shared helpers for the editor tests.

#include <doctest/doctest.h>

#include "editor_backend.h"
#include "fake_editor_backend.h"
#include "model/config_ops.h"
#include "model/selection.h"

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

}  // namespace wpiemu::app::editor::testing
