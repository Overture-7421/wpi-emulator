#pragma once

#include <QAction>
#include <QList>
#include <QWidget>

#include <functional>

#include "common/app_services.h"
#include "common/ui_resources.h"

namespace wpiemu::app {

// The two mode pages and the actions the shell forwards to them. When a page's library is not part
// of the build a placeholder takes its place, so the shell always works.
struct ShellPages {
    QWidget* editor = nullptr;
    QWidget* run = nullptr;
    bool editorIsReal = false;
    bool runIsReal = false;
    std::function<void()> saveEditor;   // File > Save
    QList<QAction*> runViewActions;     // View menu entries owned by the run page (overlays, console)
    // Registers what happens when the run view's failure overlay asks for a fix (1 = open the setup, 2 = open Settings).
    std::function<void(QObject* receiver, std::function<void(int)> handler)> onProblemAction;
};

// Creates the pages with their backend adapters; the adapters are children of `backendParent`.
ShellPages createShellPages(AppServices& services, const UiResources& ui, QObject* backendParent);

}  // namespace wpiemu::app
