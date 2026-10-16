#include "shell/page_factory.h"

#include <QLabel>
#include <QVBoxLayout>

#include "common/ui_helpers.h"

#ifdef WPIEMU_APP_HAS_EDITOR
#include "editor/editor_page.h"
#include "shell/editor_backend_adapter.h"
#endif
#ifdef WPIEMU_APP_HAS_RUNVIEW
#include "runview/run_page.h"
#include "shell/run_backend_adapter.h"
#endif

namespace wpiemu::app {

namespace {

[[maybe_unused]] QWidget* placeholder(const QString& title, const QString& message) {
    auto* w = new QWidget;
    auto* l = new QVBoxLayout(w);
    l->addStretch(1);
    auto* t = makeLabel(title, "title");
    t->setAlignment(Qt::AlignCenter);
    auto* m = makeLabel(message, "subtitle", nullptr, true);
    m->setAlignment(Qt::AlignCenter);
    l->addWidget(t);
    l->addWidget(m);
    l->addStretch(2);
    return w;
}

}  // namespace

ShellPages createShellPages([[maybe_unused]] AppServices& services, [[maybe_unused]] const UiResources& ui,
                            [[maybe_unused]] QObject* backendParent) {
    ShellPages pages;
#ifdef WPIEMU_APP_HAS_EDITOR
    {
        auto* backend = new EditorBackendAdapter(services, backendParent);
        pages.editor = new editor::EditorPage(*backend);
        pages.editorIsReal = true;
        pages.saveEditor = [backend] { backend->saveConfig(); };
    }
#else
    pages.editor = placeholder(QObject::tr("Robot editor"), QObject::tr("The 3D robot editor is not part of this build."));
#endif
#ifdef WPIEMU_APP_HAS_RUNVIEW
    {
        auto* backend = new RunBackendAdapter(services, backendParent);
        auto* runPage = new runview::RunPage(*backend);
        pages.run = runPage;
        pages.runViewActions = runPage->viewActions();
        pages.runIsReal = true;
    }
#else
    pages.run = placeholder(QObject::tr("Run view"), QObject::tr("The run view is not part of this build."));
#endif
    return pages;
}

}  // namespace wpiemu::app
