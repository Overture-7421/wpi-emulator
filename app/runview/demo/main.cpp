// Standalone demo of the run view against FakeRunBackend. Also used for headless screenshots:
//   wpiemu_runview_demo --scenario running --screenshot out.png [--size 1500x900]
// Scenarios: idle, starting, running, crash, timeout, mismatch, failed.
// Options: --light | --dark, --gamepad, --joysticks, --park (window out of sight), --still (do not drive the robot),
//   --view robot|top (frame the start area), --pose x,y,yawDeg (put the robot there), --fault photon|foreign|disabled|disconnected|unused|unconfigured|layout (cameras),
//   --hide cameras,aprilTags,... (uncheck View menu toggles by their label prefix), --crop x,y,w,h[,scale] (save just that part of the screenshot, enlarged).

#include <QAction>
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

    if (args.contains(QStringLiteral("--joysticks"))) page.driverStationPanel()->showSettings(1);

    if (scenario != QLatin1String("idle")) backend.startSimulation();
    if (scenario == QLatin1String("running")) {
        if (!args.contains(QStringLiteral("--still"))) {
            QTimer::singleShot(1200, &app, [&] {
                page.driverStation().setEnabled(true);
                page.handleKey(Qt::Key_W, true);
                page.handleKey(Qt::Key_D, true);
                page.handleKey(Qt::Key_Z, true);
                page.handleKey(Qt::Key_A, false);
            });
        } else {
            QTimer::singleShot(1200, &app, [&] { page.driverStation().setEnabled(true); });
        }
    } else if (scenario == QLatin1String("crash")) {
        QTimer::singleShot(1200, &app, [&] { backend.simulateCrash(1); });
    } else if (scenario == QLatin1String("timeout")) {
        QTimer::singleShot(300, &app, [&] { backend.simulateHandshakeTimeout(); });
    } else if (scenario == QLatin1String("mismatch")) {
        QTimer::singleShot(300, &app, [&] { backend.simulateProtocolMismatch(QStringLiteral("Extension speaks an older protocol than this app supports.")); });
    } else if (scenario == QLatin1String("failed")) {
        QTimer::singleShot(300, &app, [&] { backend.simulateFailedToStart(QStringLiteral("gradlew: command not found (JAVA_HOME is not set)")); });
    }

    // Vision: --view robot|top frames the start area instead of the whole field; --fault injects what a camera can go wrong with.
    //   --fault photon | foreign | disabled | disconnected | unused | unconfigured | layout
    const QString view = argValue(args, QStringLiteral("--view"), QStringLiteral("home"));
    QTimer::singleShot(1500, &app, [&] {
        for (const QString& name : argValue(args, QStringLiteral("--hide")).split(QLatin1Char(','), Qt::SkipEmptyParts))
            for (QAction* action : page.viewActions())
                if (action->text().startsWith(name, Qt::CaseInsensitive)) action->setChecked(false);
        if (const QStringList pose = argValue(args, QStringLiteral("--pose")).split(QLatin1Char(',')); pose.size() == 3)
            backend.setRobotPose(wpiemu::sdk::toPose3(wpiemu::sdk::StartPose{pose[0].toFloat(), pose[1].toFloat(), pose[2].toFloat() * 3.14159265f / 180.f}, 0.2f));
        wpiemu::app::OrbitCamera& orbit = page.scene().camera();
        if (view == QLatin1String("robot") || view == QLatin1String("top")) {
            orbit.target = {3.4f, 4.1f, 0.5f};
            orbit.distance = view == QLatin1String("top") ? 11.f : 7.f;
            orbit.yawRad = wpiemu::app::OrbitCamera::degToRad(view == QLatin1String("top") ? 180.f : -62.f);
            orbit.pitchRad = wpiemu::app::OrbitCamera::degToRad(view == QLatin1String("top") ? 89.f : 32.f);
            page.scene().zoom(0.f);
        }
        const QString fault = argValue(args, QStringLiteral("--fault"));
        if (fault == QLatin1String("photon")) backend.simulateCameraFault(QStringLiteral("front"), FakeRunBackend::CameraFault::PhotonUuidUnsupported);
        else if (fault == QLatin1String("foreign")) backend.simulateCameraFault(QStringLiteral("front"), FakeRunBackend::CameraFault::ForeignPublisher);
        else if (fault == QLatin1String("disabled")) backend.simulateCameraFault(QStringLiteral("front"), FakeRunBackend::CameraFault::Disabled);
        else if (fault == QLatin1String("disconnected")) backend.simulateCameraFault(QStringLiteral("front"), FakeRunBackend::CameraFault::NotConnected);
        else if (fault == QLatin1String("unused")) backend.simulateCameraFault(QStringLiteral("front"), FakeRunBackend::CameraFault::NotUsedByRobot);
        else if (fault == QLatin1String("unconfigured")) backend.simulateUnconfiguredCamera(QStringLiteral("photon-extra"));
        else if (fault == QLatin1String("layout")) backend.simulateLayoutMissing();
    });

    if (!shot.isEmpty()) {
        QTimer::singleShot(settle, &app, [&] {
            // --crop x,y,w,h[,scale] saves just that part of the page, enlarged (to look at details).
            QPixmap grabbed = page.grab();
            if (const QStringList crop = argValue(args, QStringLiteral("--crop")).split(QLatin1Char(',')); crop.size() >= 4) {
                grabbed = grabbed.copy(crop[0].toInt(), crop[1].toInt(), crop[2].toInt(), crop[3].toInt());
                const int scale = crop.size() > 4 ? crop[4].toInt() : 2;
                grabbed = grabbed.scaled(grabbed.size() * scale, Qt::KeepAspectRatio, Qt::SmoothTransformation);
            }
            const bool ok = grabbed.save(shot);
            const QImage scene = page.viewport()->grabScene();
            if (!scene.isNull()) scene.save(shot + QStringLiteral(".scene.png"));
            std::cout << "screenshot " << (ok ? "saved" : "FAILED") << " scene " << (scene.isNull() ? "null" : "ok") << " loaded "
                      << page.viewport()->sceneLoaded() << " " << page.viewport()->sceneError().toStdString() << std::endl;
            QApplication::quit();
        });
    }
    return app.exec();
}
