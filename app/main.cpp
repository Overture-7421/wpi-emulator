#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QSurfaceFormat>
#include <QTimer>

#include <cstring>
#include <iostream>

#include "common/app_services.h"
#include "common/theme.h"
#include "common/ui_resources.h"
#include "shell/main_window.h"
#include "shell/page_factory.h"
#include "shell/shell_controller.h"
#include "wpiemu/ipc/protocol.h"
#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/version.h"

#ifndef WPIEMU_APP_VERSION
#define WPIEMU_APP_VERSION "0.0.0"
#endif

namespace {

int printVersion() {
    wpiemu::sdk::GamePluginRegistry plugins;
    if (const auto status = wpiemu::sdk::registerBuiltinGamePlugins(plugins); !status.ok()) {
        std::cerr << "plugin registration failed: " << status.error().message << '\n';
        return 1;
    }
    std::cout << "wpiemu_app (sdk " << wpiemu::sdk::kSdkVersionMajor << '.' << wpiemu::sdk::kSdkVersionMinor
              << ", ipc protocol " << wpiemu::ipc::protocolVersion() << ", wpiemu_core, "
              << plugins.size() << " game plugin(s))\n";
    return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
    // Headless smoke path used by CTest; avoids creating any window.
    for (int i = 1; i < argc; ++i)
        if (std::strcmp(argv[i], "--version") == 0) return printVersion();

    // No vsync: the run view paces itself (RunPage poll timer, ~144 Hz cap).
    QSurfaceFormat format = QSurfaceFormat::defaultFormat();
    format.setSwapInterval(0);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication app(argc, argv);
    QApplication::setOrganizationName(QStringLiteral("WPI Emulator"));
    QApplication::setApplicationName(QStringLiteral("WPI Emulator"));
    QApplication::setApplicationVersion(QStringLiteral(WPIEMU_APP_VERSION));

    QCommandLineParser parser;
    parser.setApplicationDescription(QObject::tr("Run unmodified WPILib robot projects in a simulator."));
    parser.addHelpOption();
    parser.addVersionOption();
    QCommandLineOption demoOption(QStringLiteral("demo"), QObject::tr("Use simulated devices and motion instead of launching the robot."));
    QCommandLineOption projectOption(QStringLiteral("project"), QObject::tr("Open the WPILib project in <folder>."), QStringLiteral("folder"));
    parser.addOption(demoOption);
    parser.addOption(projectOption);
    parser.process(app);

    wpiemu::app::AppServicesOptions options;
    options.demo = parser.isSet(demoOption);
    wpiemu::app::AppServices services(std::move(options));
    wpiemu::app::ThemeManager theme(app, services.settings());
    wpiemu::app::UiResources ui(theme);
    wpiemu::app::ShellController controller(services);
    wpiemu::app::ShellPages pages = wpiemu::app::createShellPages(services, ui, &controller);
    wpiemu::app::MainWindow window(services, controller, ui, std::move(pages));
    window.show();

    // Start-up (after the window is visible so dialogs have a parent): explicit project, else resume the last one.
    QTimer::singleShot(0, &window, [&] {
        if (!services.startupProblems().isEmpty())
            window.showMessage(services.startupProblems().join(QStringLiteral("\n")));
        if (parser.isSet(projectOption)) window.openProjectPath(parser.value(projectOption));
        else controller.restoreLastProject();
    });
    return app.exec();
}
