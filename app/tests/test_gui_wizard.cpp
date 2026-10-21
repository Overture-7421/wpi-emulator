#include <doctest/doctest.h>

#include <QDropEvent>
#include <QFileInfo>
#include <QImage>
#include <QMimeData>
#include <QRegularExpression>
#include <QSet>
#include <QSignalSpy>
#include <QUrl>

#include <memory>

#include "common/theme.h"
#include "common/ui_helpers.h"
#include "gui_support.h"
#include "wizard/project_drop_zone.h"
#include "wizard/project_page.h"
#include "wizard/robot_page.h"
#include "wizard/summary_page.h"
#include "wizard/toolchain_page.h"
#include "wizard/wizard_dialog.h"

using namespace wpiemu::app;
using namespace wpiemu::app::testing;
namespace sdk = wpiemu::sdk;

namespace {
void enterProject(WizardDialog& dlg, const QString& path) {
    dlg.projectPage()->pathEdit()->setText(path);
    dlg.projectPage()->applyTypedPath();
}
}  // namespace

TEST_CASE("value labels wrap and let the user select the text") {
    const std::unique_ptr<QLabel> label(makeValueLabel());
    CHECK(label->wordWrap());
    CHECK(label->textInteractionFlags() == Qt::TextSelectableByMouse);
    CHECK(label->parentWidget() == nullptr);
}

TEST_CASE("wizard dialog: first-run happy path from project to a saved robot") {
    GuiEnvironment env;
    TestProject project;
    env.addProject(project);
    WizardDialog dlg(*env.services, env.ui);
    dlg.show();

    // ---- step 1: project ----
    CHECK(dlg.model().step() == WizardStep::Project);
    CHECK(!dlg.nextButton()->isEnabled());
    CHECK(!dlg.backButton()->isEnabled());
    enterProject(dlg, QStringLiteral("/definitely/missing"));
    CHECK(dlg.projectPage()->banner()->isVisible());
    CHECK(dlg.projectPage()->banner()->kind() == BannerKind::Error);
    CHECK(!dlg.nextButton()->isEnabled());

    enterProject(dlg, project.path());
    CHECK(dlg.projectPage()->banner()->kind() == BannerKind::Success);
    CHECK(dlg.nextButton()->isEnabled());

    // ---- step 2: toolchain ----
    dlg.nextButton()->click();
    CHECK(dlg.model().step() == WizardStep::Toolchain);
    REQUIRE(waitFor([&] { return dlg.model().toolchainChecked(); }));
    CHECK(dlg.toolchainPage()->rowCount() == 5);
    CHECK(dlg.toolchainPage()->rowStatus(QStringLiteral("jdk")) == CheckStatus::Ok);
    CHECK(dlg.toolchainPage()->rowStatus(QStringLiteral("gradle")) == CheckStatus::Ok);
    CHECK(dlg.toolchainPage()->rowStatus(QStringLiteral("cpp")) == CheckStatus::NotApplicable);
    CHECK(dlg.toolchainPage()->rowStatus(QStringLiteral("game")) == CheckStatus::Ok);
    CHECK(dlg.toolchainPage()->summaryBanner()->kind() == BannerKind::Success);
    CHECK(dlg.backButton()->isEnabled());

    // ---- step 3: robot ----
    dlg.nextButton()->click();
    REQUIRE(dlg.model().step() == WizardStep::Robot);
    RobotPage* robot = dlg.robotPage();
    CHECK(robot->schematic()->model().drivetrain.spec.wheels.size() == 4);
    robot->drivetrainButton(sdk::drivetrain_kind::kTank)->click();
    CHECK(dlg.model().basics().drivetrain == sdk::drivetrain_kind::kTank);
    CHECK(robot->schematic()->model().drivetrain.spec.wheels.size() == 6);
    CHECK(robot->wheelsPerSideSpin()->isVisibleTo(robot));
    robot->wheelsPerSideSpin()->setValue(4);
    CHECK(robot->schematic()->model().drivetrain.spec.wheels.size() == 8);

    robot->nameEdit()->setText(QStringLiteral("Gui bot"));
    emit robot->nameEdit()->textEdited(QStringLiteral("Gui bot"));
    robot->massSpin()->setValue(110.0);   // lb (imperial is the default)
    CHECK(dlg.model().basics().massKg == doctest::Approx(49.9).epsilon(0.01));
    CHECK(dlg.model().basics().name == QStringLiteral("Gui bot"));

    robot->massSpin()->setValue(robot->massSpin()->minimum());   // still valid
    robot->massSpin()->setValue(300.0);   // clamped by the spin box range, never invalid
    CHECK(dlg.model().basicsValid());
    robot->massSpin()->setValue(140.0);   // over the 125 lb limit: allowed, with a warning
    CHECK(dlg.model().basicsValid());
    CHECK(robot->banner()->kind() == BannerKind::Warning);
    CHECK(dlg.nextButton()->isEnabled());
    robot->massSpin()->setValue(120.0);
    CHECK(!robot->banner()->isVisible());

    // the preview paints something meaningful
    const QImage img = robot->schematic()->grab().toImage();
    QSet<QRgb> colours;
    for (int y = 0; y < img.height(); y += 7)
        for (int x = 0; x < img.width(); x += 7) colours.insert(img.pixel(x, y));
    CHECK(colours.size() > 4);

    // ---- step 4: summary ----
    dlg.nextButton()->click();
    REQUIRE(dlg.model().step() == WizardStep::Summary);
    CHECK(dlg.nextButton()->text() == QStringLiteral("Open editor"));
    SummaryPage* summary = dlg.summaryPage();
    CHECK(summary->detectCheck()->isEnabled());
    CHECK(summary->detectCheck()->isChecked());
    CHECK(summary->robotSummary().contains(QStringLiteral("Gui bot")));
    CHECK(summary->robotSummary().contains(QStringLiteral("tank")));
    CHECK(summary->gameSummary().contains(QStringLiteral("2026")));

    CHECK(!env.services->configs().exists(project.path()));   // nothing is written before the last click
    dlg.nextButton()->click();
    CHECK(dlg.result() == QDialog::Accepted);
    CHECK(dlg.wizardResult().ok);
    CHECK(dlg.wizardResult().detectDevices);
    CHECK(QFileInfo::exists(project.path() + "/.wpiemu/robot.json"));
    REQUIRE(env.services->session().hasConfig());
    CHECK(env.services->session().config()->name == "Gui bot");
    CHECK(env.services->session().config()->drivetrain.spec.wheels.size() == 8);
}

TEST_CASE("wizard dialog: cancel is always safe") {
    GuiEnvironment env;
    TestProject project;
    env.addProject(project);
    {
        WizardDialog dlg(*env.services, env.ui);
        dlg.show();
        enterProject(dlg, project.path());
        dlg.nextButton()->click();
        waitFor([&] { return dlg.model().toolchainChecked(); });
        dlg.nextButton()->click();
        dlg.cancelButton()->click();
        CHECK(dlg.result() == QDialog::Rejected);
    }
    CHECK(!env.services->configs().exists(project.path()));
    CHECK(!QFileInfo::exists(project.path() + "/.wpiemu"));
    CHECK(!env.services->session().hasProject());
}

TEST_CASE("wizard dialog: sidebar steps unlock as they are completed") {
    GuiEnvironment env;
    TestProject project;
    env.addProject(project);
    WizardDialog dlg(*env.services, env.ui);
    dlg.show();
    CHECK(dlg.stepButton(WizardStep::Project)->isEnabled());
    CHECK(!dlg.stepButton(WizardStep::Robot)->isEnabled());
    enterProject(dlg, project.path());
    REQUIRE(waitFor([&] { return dlg.model().toolchainChecked(); }));
    CHECK(dlg.stepButton(WizardStep::Robot)->isEnabled());
    dlg.stepButton(WizardStep::Robot)->click();
    CHECK(dlg.model().step() == WizardStep::Robot);
    dlg.stepButton(WizardStep::Project)->click();
    CHECK(dlg.model().step() == WizardStep::Project);
}

TEST_CASE("wizard dialog: missing tools are explained with a fix and do not block") {
    GuiEnvironment env(/*healthyMachine=*/false);
    TestProject project(QStringLiteral("cpp"), 2031);
    env.addProject(project);
    WizardDialog dlg(*env.services, env.ui);
    dlg.show();
    enterProject(dlg, project.path());
    dlg.nextButton()->click();
    REQUIRE(waitFor([&] { return dlg.model().toolchainChecked(); }));
    CHECK(dlg.toolchainPage()->rowStatus(QStringLiteral("jdk")) == CheckStatus::Missing);
    CHECK(dlg.toolchainPage()->rowStatus(QStringLiteral("cpp")) == CheckStatus::Missing);
    CHECK(dlg.toolchainPage()->rowStatus(QStringLiteral("game")) == CheckStatus::Warning);
    CHECK(dlg.toolchainPage()->summaryBanner()->kind() == BannerKind::Warning);
    CHECK(dlg.nextButton()->isEnabled());

    // fixing the machine and re-checking turns the rows green
    env.probe->addJava(QStringLiteral("/jdk/bin/java"), QStringLiteral("openjdk version \"21.0.1\""));
    env.probe->environment[QStringLiteral("JAVA_HOME")] = QStringLiteral("/jdk");
    env.probe->onPath[QStringLiteral("g++")] = QStringLiteral("/usr/bin/g++");
    dlg.toolchainPage()->recheckButton()->click();
    REQUIRE(waitFor([&] { return dlg.model().toolchainChecked() && !dlg.model().toolchainRunning(); }));
    CHECK(dlg.toolchainPage()->rowStatus(QStringLiteral("jdk")) == CheckStatus::Ok);
    CHECK(dlg.toolchainPage()->rowStatus(QStringLiteral("cpp")) == CheckStatus::Ok);
    CHECK(dlg.toolchainPage()->rowStatus(QStringLiteral("game")) == CheckStatus::Warning);   // still core-only
}

TEST_CASE("wizard dialog: dropping a folder or a file inside it selects the project") {
    GuiEnvironment env;
    TestProject project;
    env.addProject(project);
    WizardDialog dlg(*env.services, env.ui);
    dlg.show();
    ProjectDropZone* zone = dlg.projectPage()->dropZone();
    QSignalSpy dropped(zone, &ProjectDropZone::pathDropped);

    QMimeData mime;
    mime.setUrls({QUrl::fromLocalFile(project.path() + "/build.gradle")});
    CHECK(ProjectDropZone::pathFromMimeData(&mime) == project.path() + "/build.gradle");
    CHECK(zone->acceptPayload(&mime));
    CHECK(dropped.count() == 1);
    CHECK(dlg.model().projectValid());
    CHECK(QDir::cleanPath(dlg.model().projectPath()) == project.path());   // resolved to the folder

    QMimeData text;
    text.setText(QStringLiteral("not a file"));
    CHECK(ProjectDropZone::pathFromMimeData(&text).isEmpty());
    CHECK(!zone->acceptPayload(&text));
    CHECK(ProjectDropZone::pathFromMimeData(nullptr).isEmpty());
}

TEST_CASE("wizard dialog: recent projects can be picked from the first page") {
    GuiEnvironment env;
    TestProject project;
    env.addProject(project);
    env.services->recentProjects().touch(project.path());
    WizardDialog dlg(*env.services, env.ui);
    dlg.show();
    QListWidget* recent = dlg.projectPage()->recentList();
    REQUIRE(recent->count() == 1);
    emit recent->itemClicked(recent->item(0));
    CHECK(dlg.model().projectValid());
    CHECK(dlg.nextButton()->isEnabled());
}

TEST_CASE("theme: style sheet is fully resolved for light and dark") {
    for (const ThemeTokens& tokens : {lightTokens(), darkTokens()}) {
        const QString sheet = buildStyleSheet(tokens);
        CHECK(sheet.contains(tokens.accent.name()));
        CHECK(!sheet.contains(QRegularExpression(QStringLiteral("%[a-zA-Z]+%"))));
        const QPalette p = makePalette(tokens);
        CHECK(p.color(QPalette::Window) == tokens.window);
        CHECK(p.color(QPalette::Highlight) == tokens.accent);
    }
    CHECK(lightTokens().window != darkTokens().window);
    CHECK(!lightTokens().dark);
    CHECK(darkTokens().dark);
}
