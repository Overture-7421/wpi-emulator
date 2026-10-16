#pragma once

#include <QApplication>

#include "common/theme.h"
#include "common/ui_resources.h"
#include "test_support.h"

namespace wpiemu::app::testing {

// Test environment plus the theme and icon resources every view needs.
struct GuiEnvironment : TestEnvironment {
    ThemeManager theme;
    UiResources ui;

    explicit GuiEnvironment(bool healthyMachine = true, bool realRuntime = false)
        : TestEnvironment(healthyMachine, realRuntime),
          theme(static_cast<QApplication&>(*QCoreApplication::instance()), services->settings()),
          ui(theme) {}
};

}  // namespace wpiemu::app::testing
