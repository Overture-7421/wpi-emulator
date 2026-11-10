// wpiemu_client: the thin game client of WPI Emulator practice matches. It joins a match the desktop app hosts (over the relay or
// directly), predicts its own robot and draws the shared scene. No WPILib, JDK or Gradle.
//
//   wpiemu_client                      the client window
//   wpiemu_client --version            prints the version and exits (no window; used by CTest and the installer checks)
//   wpiemu_client --demo [--scenario NAME] [--screenshot FILE] [--size WxH] [--settle MS] [--park] [--stats]
//                                      a scripted match, nothing leaves the process; with --screenshot saves the window (and the 3D
//                                      scene next to it as FILE.scene.png) after --settle ms and exits
//
// Scenarios: join, join-errors, lobby, live, paused, stalled, reconnecting, lost, kicked, closed.

#include <QApplication>
#include <QDir>
#include <QFile>
#include <QGuiApplication>
#include <QLineEdit>
#include <QPushButton>
#include <QSettings>
#include <QStyleHints>
#include <QTemporaryDir>
#include <QTimer>
#include <QToolButton>

#include <cstring>
#include <iostream>

#include "client/client_module.h"
#include "client/client_window.h"
#include "client/demo_session.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/core/config/robot_config_codec.h"
#include "wpiemu/net/module.h"
#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/version.h"

#ifndef WPIEMU_APP_VERSION
#define WPIEMU_APP_VERSION "0.0.0"
#endif

namespace {

using namespace wpiemu;
namespace cl = wpiemu::app::client;

// Time for the scene to draw before a scenario is captured.
constexpr int kDefaultSettleMs = 2500;
constexpr int kParkedWindowX = -6000;   // --park moves the window off every screen so screenshots work on a real platform

int printVersion() {
    sdk::GamePluginRegistry plugins;
    if (const auto status = sdk::registerBuiltinGamePlugins(plugins); !status.ok()) {
        std::cerr << "plugin registration failed: " << status.error().message << '\n';
        return 1;
    }
    std::cout << cl::moduleName() << ' ' << WPIEMU_APP_VERSION << " (sdk " << sdk::kSdkVersionMajor << '.' << sdk::kSdkVersionMinor
              << ", " << net::moduleName() << ", wpiemu_core, " << plugins.size() << " game plugin(s))\n";
    return 0;
}

QString argValue(const QStringList& args, const QString& name, const QString& fallback = {}) {
    const int i = args.indexOf(name);
    return i >= 0 && i + 1 < args.size() ? args[i + 1] : fallback;
}

// A robot.json the demo's join screen can show: heavier than FRC allows, so the host's corrections are listed.
QString writeDemoRobot(const QString& folder, const cl::ClientServices& services) {
    sdk::RobotConfig robot = cl::demoRobotConfig(services);
    robot.name = "Demo Bot";
    robot.chassis.massKg = 160.f;
    const QString path = QDir(folder).filePath(QStringLiteral("robot.json"));
    QFile file(path);
    if (file.open(QIODevice::WriteOnly)) {
        const core::config::RobotConfigCodec codec;
        file.write(QByteArray::fromStdString(core::config::JsonValueCodec().dump(codec.toValue(robot), true)));
    }
    return path;
}

int runWindow(QApplication& app, const QStringList& args, bool demo) {
    cl::ClientWindowOptions options;
    QTemporaryDir demoFolder;
    QString scenarioName = argValue(args, QStringLiteral("--scenario"), QStringLiteral("live"));
    std::optional<cl::DemoScenario> scenario;
    std::unique_ptr<cl::ClientServices> demoServices;
    if (demo) {
        const bool errors = scenarioName == QLatin1String("join-errors");
        scenario = cl::parseDemoScenario((errors ? QStringLiteral("join") : scenarioName).toStdString());
        if (!scenario) {
            std::cerr << "unknown --scenario " << scenarioName.toStdString() << " (" << cl::demoScenarioNames() << ", join-errors)\n";
            return 2;
        }
        auto services = cl::ClientServices::create();
        if (!services) {
            std::cerr << "client setup failed: " << services.error().message << '\n';
            return 1;
        }
        demoServices = std::move(services).value();
        options.settingsFile = demoFolder.filePath(QStringLiteral("demo.ini"));   // the demo never touches the user's settings
        options.session = std::make_unique<cl::DemoClientSession>(*demoServices, *scenario);
        options.useGamepads = args.contains(QStringLiteral("--gamepad"));
    }

    cl::ClientWindow window(std::move(options));
    const QStringList wh = argValue(args, QStringLiteral("--size"), QStringLiteral("1500x900")).split(QLatin1Char('x'));
    if (wh.size() == 2) window.resize(wh[0].toInt(), wh[1].toInt());
    if (args.contains(QStringLiteral("--park"))) window.move(kParkedWindowX, 100);
    window.show();

    if (demo && *scenario == cl::DemoScenario::Join) {
        cl::JoinPage* page = window.joinPage();
        page->setInvite(scenarioName == QLatin1String("join-errors") ? QStringLiteral("ABCDE-12") : QStringLiteral("K7M2P-9QXRT-4WH8D-J3NVB"));
        page->nameEdit()->setText(QStringLiteral("Iron Owls"));
        if (scenarioName == QLatin1String("join-errors")) {
            page->robotEdit()->clear();
            page->directEdit()->setText(QStringLiteral("bad host:1"));
            page->advancedToggle()->setChecked(true);
        } else {
            window.controller().chooseRobot(writeDemoRobot(demoFolder.path(), *demoServices));
        }
        if (scenarioName == QLatin1String("join-errors")) page->joinButton()->click();
    }
    if (args.contains(QStringLiteral("--stats")) && window.showingDrive()) window.drivePage()->setDiagnosticsVisible(true);
    if (argValue(args, QStringLiteral("--camera")) == QLatin1String("home") && window.showingDrive())
        QTimer::singleShot(200, &window, [&window] { window.drivePage()->scene().resetCamera(); });

    const QString shot = argValue(args, QStringLiteral("--screenshot"));
    if (!shot.isEmpty()) {
        QTimer::singleShot(argValue(args, QStringLiteral("--settle"), QString::number(kDefaultSettleMs)).toInt(), &app, [&] {
            if (args.contains(QStringLiteral("--stats")) && window.showingDrive()) window.drivePage()->setDiagnosticsVisible(true);
            const bool ok = window.grab().save(shot);
            bool sceneOk = false;
            if (window.showingDrive()) {
                const QImage scene = window.drivePage()->viewport()->grabScene();
                sceneOk = !scene.isNull() && scene.save(shot + QStringLiteral(".scene.png"));
            }
            std::cout << "screenshot " << (ok ? "saved" : "FAILED") << " scene " << (sceneOk ? "ok" : "none") << std::endl;
            QApplication::quit();
        });
    }
    return app.exec();
}

}  // namespace

int main(int argc, char* argv[]) {
    // Headless smoke path used by CTest; avoids creating any window.
    for (int i = 1; i < argc; ++i)
        if (std::strcmp(argv[i], "--version") == 0) return printVersion();

    QApplication app(argc, argv);
    QApplication::setOrganizationName(QStringLiteral("WpiEmulator"));
    QApplication::setApplicationName(QStringLiteral("wpiemu_client"));
    QApplication::setApplicationVersion(QStringLiteral(WPIEMU_APP_VERSION));
    const QStringList args = app.arguments();
    if (args.contains(QStringLiteral("--light"))) QGuiApplication::styleHints()->setColorScheme(Qt::ColorScheme::Light);
    if (args.contains(QStringLiteral("--dark"))) QGuiApplication::styleHints()->setColorScheme(Qt::ColorScheme::Dark);
    return runWindow(app, args, args.contains(QStringLiteral("--demo")));
}
