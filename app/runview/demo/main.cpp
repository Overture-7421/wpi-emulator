// Standalone demo of the run view against FakeRunBackend. Also used for headless screenshots:
//   wpiemu_runview_demo --scenario running|practice|... --camera driver --screenshot out.png [--size 1500x900] [--state]
// --state opens the Robot state window (and saves it next to the screenshot as out.png.state.png).
// --camera accepts home, driver, blue1..blue3 or red1..red3; --no-follow disables Driver POV tracking.
// Scenarios: idle, starting, running, crash, timeout, mismatch, failed.

#include <QApplication>
#include <QDir>
#include <QGuiApplication>
#include <QStyleHints>
#include <QMainWindow>
#include <QSettings>
#include <QTabWidget>
#include <QTemporaryDir>
#include <QTimer>

#include <iostream>

#include "fake_joystick.h"
#include "fake_run_backend.h"
#include "run_page.h"

using namespace wpiemu::app::runview;

namespace {

// Time for the simulation to be running and drawn before a scenario acts on it.
constexpr int kScenarioActionDelayMs = 1200;
// Time before an injected startup failure; the page shows Starting until then.
constexpr int kStartupFaultDelayMs = 300;
// Startup delay that keeps the "starting" scenario in Starting for the whole capture.
constexpr int kHeldStartupDelayMs = 60000;
// Where --park moves the window: off every screen, so screenshots work on a real platform.
constexpr int kParkedWindowX = -6000;

QString argValue(const QStringList& args, const QString& name, const QString& fallback = {}) {
    const int i = args.indexOf(name);
    return i >= 0 && i + 1 < args.size() ? args[i + 1] : fallback;
}

}  // namespace

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("wpiemu_runview_demo"));
    const QStringList args = app.arguments();
    if (args.contains(QStringLiteral("--light"))) QGuiApplication::styleHints()->setColorScheme(Qt::ColorScheme::Light);
    if (args.contains(QStringLiteral("--dark"))) QGuiApplication::styleHints()->setColorScheme(Qt::ColorScheme::Dark);
    const QString scenario = argValue(args, QStringLiteral("--scenario"), QStringLiteral("running"));
    const QString shot = argValue(args, QStringLiteral("--screenshot"));
    const QString size = argValue(args, QStringLiteral("--size"), QStringLiteral("1500x900"));
    const int settle = argValue(args, QStringLiteral("--settle"), QStringLiteral("2500")).toInt();
    const QString camera = argValue(args, QStringLiteral("--camera"));

    QTemporaryDir settingsDir;
    QSettings settings(settingsDir.filePath(QStringLiteral("demo.ini")), QSettings::IniFormat);
    if (!camera.isEmpty()) {
        if (!parseCameraViewKey(camera.toStdString())) {
            std::cerr << "unknown --camera view: " << camera.toStdString() << std::endl;
            return 2;
        }
        settings.setValue(QStringLiteral("runview/camera/view"), camera);
    }
    if (args.contains(QStringLiteral("--no-follow")))
        settings.setValue(QStringLiteral("runview/camera/follow"), false);

    FakeRunBackend::Options options;
    if (scenario == QLatin1String("starting")) options.startupDelayMs = kHeldStartupDelayMs;
    FakeRunBackend backend(options);
    FakeJoystickProvider gamepads;
    if (args.contains(QStringLiteral("--gamepad"))) gamepads.addGamepad(QStringLiteral("Demo Gamepad"));

    RunPageOptions pageOptions;
    pageOptions.settings = &settings;
    pageOptions.gamepads = &gamepads;
    RunPage page(backend, pageOptions);

    QMainWindow window;
    window.setWindowTitle(QStringLiteral("Run view demo (fake backend)"));
    window.setCentralWidget(&page);   // QMainWindow takes ownership, but `page` lives on the stack: handed back before exit
    const QStringList wh = size.split(QLatin1Char('x'));
    window.resize(wh.value(0).toInt(), wh.value(1).toInt());
    if (args.contains(QStringLiteral("--park"))) window.move(kParkedWindowX, 100);   // real platform, window out of sight (screenshots)
    window.show();

    if (args.contains(QStringLiteral("--joysticks"))) page.driverStationPanel()->showSettings(1);
    if (args.contains(QStringLiteral("--state"))) page.stateAction()->setChecked(true);

    if (scenario != QLatin1String("idle")) backend.startSimulation();
    if (scenario == QLatin1String("running")) {
        QTimer::singleShot(kScenarioActionDelayMs, &app, [&] {
            page.driverStation().setEnabled(true);
            page.handleKey(Qt::Key_W, true);
            page.handleKey(Qt::Key_D, true);
            page.handleKey(Qt::Key_Z, true);
            page.handleKey(Qt::Key_A, false);
        });
    } else if (scenario == QLatin1String("practice")) {
        QTimer::singleShot(kScenarioActionDelayMs, &app, [&] {
            page.driverStation().setDsMode(DsMode::Practice);
            page.driverStation().setEnabled(true);
        });
    } else if (scenario == QLatin1String("crash")) {
        QTimer::singleShot(kScenarioActionDelayMs, &app, [&] { backend.simulateCrash(1); });
    } else if (scenario == QLatin1String("timeout")) {
        QTimer::singleShot(kStartupFaultDelayMs, &app, [&] { backend.simulateHandshakeTimeout(); });
    } else if (scenario == QLatin1String("mismatch")) {
        QTimer::singleShot(kStartupFaultDelayMs, &app, [&] { backend.simulateProtocolMismatch(QStringLiteral("Extension speaks an older protocol than this app supports.")); });
    } else if (scenario == QLatin1String("failed")) {
        QTimer::singleShot(kStartupFaultDelayMs, &app, [&] { backend.simulateFailedToStart(QStringLiteral("gradlew: command not found (JAVA_HOME is not set)")); });
    }

    if (!shot.isEmpty()) {
        QTimer::singleShot(settle, &app, [&] {
            const bool ok = page.grab().save(shot);
            if (page.stateWindow()->isVisible()) page.stateWindow()->grab().save(shot + QStringLiteral(".state.png"));
            const QImage scene = page.viewport()->grabScene();
            if (!scene.isNull()) scene.save(shot + QStringLiteral(".scene.png"));
            std::cout << "screenshot " << (ok ? "saved" : "FAILED") << " scene " << (scene.isNull() ? "null" : "ok") << " loaded "
                      << page.viewport()->sceneLoaded() << " " << page.viewport()->sceneError().toStdString() << std::endl;
            QApplication::quit();
        });
    }
    const int exitCode = app.exec();
    window.takeCentralWidget();   // otherwise the window's destructor deletes the stack-allocated page (heap corruption, 0xC0000374)
    return exitCode;
}
