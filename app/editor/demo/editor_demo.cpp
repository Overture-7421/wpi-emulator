// Standalone demo of the robot editor on top of FakeEditorBackend:
//   wpiemu_editor_demo                       interactive window
//   wpiemu_editor_demo --grab out.png        render once, save a PNG of the whole page, exit
//   wpiemu_editor_demo --select intake       select an item first (chassis, drivetrain, wheel:fl, slot:intake.motor, zone:pickup, plugin)
//   wpiemu_editor_demo --detect              run the scripted "Detect devices" before grabbing
//   wpiemu_editor_demo --tank                tank drive with 3 wheels per side, 3 motors per side (linked wheels)
//   wpiemu_editor_demo --light               light palette
#include <QApplication>
#include <QCommandLineParser>
#include <QPalette>
#include <QStyleFactory>
#include <QTimer>

#include "editor_page.h"
#include "fake_editor_backend.h"
#include "model/motion_links.h"
#include "wpiemu/sdk/mechanisms/motion_config.h"
#include "model/selection.h"
#include "viewmodel/editor_controller.h"

using namespace wpiemu::app::editor;

namespace {

void applyPalette(QApplication& app, bool light) {
    app.setStyle(QStyleFactory::create(QStringLiteral("Fusion")));
    QPalette p;
    if (light) {
        p.setColor(QPalette::Window, QColor("#f3f4f6"));
        p.setColor(QPalette::WindowText, QColor("#1f2328"));
        p.setColor(QPalette::Base, QColor("#ffffff"));
        p.setColor(QPalette::AlternateBase, QColor("#f0f2f5"));
        p.setColor(QPalette::Text, QColor("#1f2328"));
        p.setColor(QPalette::Button, QColor("#e6e8eb"));
        p.setColor(QPalette::ButtonText, QColor("#1f2328"));
        p.setColor(QPalette::Highlight, QColor("#0a6cff"));
        p.setColor(QPalette::HighlightedText, QColor("#ffffff"));
        p.setColor(QPalette::PlaceholderText, QColor("#6e7781"));
    } else {
        p.setColor(QPalette::Window, QColor("#22262c"));
        p.setColor(QPalette::WindowText, QColor("#d7dae0"));
        p.setColor(QPalette::Base, QColor("#1b1e23"));
        p.setColor(QPalette::AlternateBase, QColor("#20242a"));
        p.setColor(QPalette::Text, QColor("#d7dae0"));
        p.setColor(QPalette::Button, QColor("#2d323a"));
        p.setColor(QPalette::ButtonText, QColor("#d7dae0"));
        p.setColor(QPalette::Highlight, QColor("#2f6feb"));
        p.setColor(QPalette::HighlightedText, QColor("#ffffff"));
        p.setColor(QPalette::PlaceholderText, QColor("#7d8590"));
        p.setColor(QPalette::Mid, QColor("#3a404a"));
        p.setColor(QPalette::ToolTipBase, QColor("#2d323a"));
        p.setColor(QPalette::ToolTipText, QColor("#d7dae0"));
        p.setColor(QPalette::Disabled, QPalette::WindowText, QColor("#8b949e"));
        p.setColor(QPalette::Disabled, QPalette::Text, QColor("#8b949e"));
    }
    app.setPalette(p);
}

// Six wheels, one powered wheel per side (three motors each, the right side reversed); the other wheels follow it.
wpiemu::sdk::RobotConfig tankConfig() {
    namespace sdk = wpiemu::sdk;
    sdk::RobotConfig c = FakeEditorBackend::sampleConfig();
    c.name = "Tank Robot";
    c.drivetrain.spec.kind = sdk::drivetrain_kind::kTank;
    c.drivetrain.spec.wheels.clear();
    c.drivetrain.mappings.clear();
    int canId = 1;
    for (const char side : {'l', 'r'})
        for (int i = 0; i < 3; ++i) {
            sdk::WheelSpec w;
            w.name = std::string(1, side) + std::to_string(i + 1);
            w.positionM = {0.28f - 0.28f * static_cast<float>(i), side == 'l' ? 0.28f : -0.28f, w.radiusM};
            c.drivetrain.spec.wheels.push_back(w);
            if (i == 0) {
                c.drivetrain.spec.wheels.back().driveMotorCount = 3;
                for (int m = 0; m < 3; ++m)
                    c.drivetrain.mappings.push_back({{sdk::slot_owner::kDrivetrain, sdk::driveSlotName(w.name, m)},
                                                     {"phoenix6", "talonfx", canId++, "canivore", -1},
                                                     side == 'r'});
            }
        }
    c.motionLinks.clear();
    c.motionLinks = sdk::tankMotionLinks(c).value();
    return c;
}

SelectionRef parseSelection(const QString& text) {
    if (text == QStringLiteral("chassis")) return SelectionRef::chassis();
    if (text == QStringLiteral("drivetrain")) return SelectionRef::drivetrain();
    if (text == QStringLiteral("plugin")) return SelectionRef::plugin();
    if (text.startsWith(QStringLiteral("zone:"))) return SelectionRef::zone(text.mid(5).toStdString());
    if (text.startsWith(QStringLiteral("wheel:"))) return SelectionRef::wheel(text.mid(6).toStdString());
    if (text.startsWith(QStringLiteral("slot:"))) {
        const QString rest = text.mid(5);
        const int dot = static_cast<int>(rest.indexOf(QLatin1Char('.')));
        return SelectionRef::slot(rest.left(dot).toStdString(), rest.mid(dot + 1).toStdString());
    }
    return SelectionRef::mechanism(text.toStdString());
}

}  // namespace

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    QCommandLineParser parser;
    parser.addHelpOption();
    const QCommandLineOption grab(QStringLiteral("grab"), QStringLiteral("Save a PNG of the page and exit."), QStringLiteral("file"));
    const QCommandLineOption select(QStringLiteral("select"), QStringLiteral("Select an item first."), QStringLiteral("item"));
    const QCommandLineOption detect(QStringLiteral("detect"), QStringLiteral("Run the scripted device detection before grabbing."));
    const QCommandLineOption tank(QStringLiteral("tank"), QStringLiteral("Use a tank drive with linked wheels."));
    const QCommandLineOption light(QStringLiteral("light"), QStringLiteral("Use the light palette."));
    const QCommandLineOption size(QStringLiteral("size"), QStringLiteral("Window size WxH."), QStringLiteral("size"), QStringLiteral("1500x900"));
    parser.addOptions({grab, select, detect, tank, light, size});
    parser.process(app);

    applyPalette(app, parser.isSet(light));
    FakeEditorBackend backend;
    if (parser.isSet(tank)) backend.setConfig(tankConfig());
    EditorPage page(backend);
    page.setWindowTitle(QStringLiteral("WPI Emulator - robot editor (demo)"));
    const QStringList wh = parser.value(size).split(QLatin1Char('x'));
    page.resize(wh.value(0).toInt(), wh.value(1).toInt());
    page.show();

    if (parser.isSet(select)) page.controller().selection().set(parseSelection(parser.value(select)));
    if (parser.isSet(detect)) {
        backend.setDetectionStepMs(60);
        backend.startDetection();
    }
    if (parser.isSet(grab)) {
        const QString path = parser.value(grab);
        QTimer::singleShot(parser.isSet(detect) ? 1800 : 1200, [&page, path, &app] {
            page.grab().save(path);
            app.quit();
        });
    }
    return app.exec();
}
