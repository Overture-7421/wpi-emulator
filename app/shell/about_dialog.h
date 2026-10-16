#pragma once

#include <QDialog>
#include <QString>

#include "common/ui_resources.h"
#include "wpiemu/sdk/game/plugin_registry.h"

namespace wpiemu::app {

#ifndef WPIEMU_APP_VERSION
#define WPIEMU_APP_VERSION "0.0.0"
#endif

// Plain-text build/environment details (also what "Copy details" puts on the clipboard).
QString aboutDetails(const sdk::GamePluginRegistry& plugins);

class AboutDialog : public QDialog {
    Q_OBJECT
public:
    AboutDialog(const sdk::GamePluginRegistry& plugins, const UiResources& ui, QWidget* parent = nullptr);
};

}  // namespace wpiemu::app
