// Standalone demo of the run view against FakeRunBackend. Also used for headless screenshots:
//   wpiemu_runview_demo --scenario running --screenshot out.png [--camera follow] [--size 1500x900]
// Scenarios: idle, starting, running, crash, timeout, mismatch, failed.

#include <QApplication>
#include <QDir>
#include <QGuiApplication>
#include <QStyleHints>
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
    const QString camera = argValue(args, QStringLiteral("--camera"), QStringLiteral("orbit"));
    const QString size = argValue(args, QStringLiteral("--size"), QStringLiteral("1500x900"));
    const int settle = argValue(args, QStringLiteral("--settle"), QStringLiteral("2500")).toInt();

    QTemporaryDir settingsDir;
    QSettings settings(settingsDir.filePath(QStringLiteral("demo.ini")), QSettings::IniFormat);

    FakeRunBackend::Options options;
    options.startupDelayMs = scenario == QLatin1String("starting") ? 60000 : 350;
    FakeRunBackend backend(options);
    FakeJoystickProvider gamepads;
    if (args.contains(QStringLiteral("--gamepad"))) gamepads.addGamepad(QStringLiteral("Demo Gamepad"));

    RunPageOptions pageOptions;
    pageOptions.settings = &settings;
    pageOptions.gamepads = &gamepads;
    RunPage page(backend, pageOptions);

    QMainWindow window;
    window.setWindowTitle(QStringLiteral("Run view demo (fake backend)"));
    window.setCentralWidget(&page);
    const QStringList wh = size.split(QLatin1Char('x'));
    window.resize(wh.value(0).toInt(), wh.value(1).toInt());
    if (args.contains(QStringLiteral("--park"))) window.move(-6000, 100);   // real platform, window out of sight (screenshots)
    window.show();

    const int cam = camera == QLatin1String("follow") ? 1 : (camera == QLatin1String("top") ? 2 : 0);
    page.setCameraMode(cam);
    if (args.contains(QStringLiteral("--joysticks"))) page.driverStationPanel()->findChild<QTabWidget*>()->setCurrentIndex(1);

    if (scenario != QLatin1String("idle")) backend.startRobot();
    if (scenario == QLatin1String("running")) {
        QTimer::singleShot(1200, &app, [&] {
            page.driverStation().setEnabled(true);
            page.handleKey(Qt::Key_W, true);
            page.handleKey(Qt::Key_D, true);
            page.handleKey(Qt::Key_Z, true);
            page.handleKey(Qt::Key_A, false);
        });
    } else if (scenario == QLatin1String("crash")) {
        QTimer::singleShot(1200, &app, [&] { backend.simulateCrash(1); });
    } else if (scenario == QLatin1String("timeout")) {
        QTimer::singleShot(300, &app, [&] { backend.simulateHandshakeTimeout(); });
    } else if (scenario == QLatin1String("mismatch")) {
        QTimer::singleShot(300, &app, [&] { backend.simulateProtocolMismatch(QStringLiteral("Extension speaks protocol v1, this app needs v2 or newer.")); });
    } else if (scenario == QLatin1String("failed")) {
        QTimer::singleShot(300, &app, [&] { backend.simulateFailedToStart(QStringLiteral("gradlew: command not found (JAVA_HOME is not set)")); });
    }

    if (!shot.isEmpty()) {
        QTimer::singleShot(settle, &app, [&] {
            const bool ok = page.grab().save(shot);
            const QImage scene = page.viewport()->grabScene();
            if (!scene.isNull()) scene.save(shot + QStringLiteral(".scene.png"));
            std::cout << "screenshot " << (ok ? "saved" : "FAILED") << " scene " << (scene.isNull() ? "null" : "ok") << " loaded "
                      << page.viewport()->sceneLoaded() << " " << page.viewport()->sceneError().toStdString() << std::endl;
            QApplication::quit();
        });
    }
    return app.exec();
}
