#include "wizard/summary_page.h"

#include <QGridLayout>
#include <QSignalBlocker>
#include <QVBoxLayout>

namespace wpiemu::app {

SummaryPage::SummaryPage(WizardViewModel& model, const UiResources& ui, QWidget* parent)
    : WizardPage(parent),
      model_(model),
      ui_(ui),
      projectValue_(makeValueLabel()),
      gameValue_(makeValueLabel()),
      toolchainValue_(makeValueLabel()),
      robotValue_(makeValueLabel()),
      locationValue_(makeValueLabel()),
      detectCheck_(new QCheckBox(tr("Detect devices now"))),
      detectHint_(makeLabel({}, "muted", nullptr, true)) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(14);

    QFrame* card = makeCard();
    auto* grid = new QGridLayout(card);
    grid->setContentsMargins(18, 16, 18, 16);
    grid->setHorizontalSpacing(20);
    grid->setVerticalSpacing(10);
    grid->setColumnStretch(1, 1);
    const std::pair<QString, QLabel*> rows[] = {{tr("Project"), projectValue_},
                                                {tr("Game"), gameValue_},
                                                {tr("Toolchain"), toolchainValue_},
                                                {tr("Robot"), robotValue_},
                                                {tr("Saved to"), locationValue_}};
    int row = 0;
    for (const auto& [name, value] : rows) {
        grid->addWidget(makeLabel(name, "muted"), row, 0, Qt::AlignTop);
        grid->addWidget(value, row, 1);
        ++row;
    }
    root->addWidget(card);

    QFrame* detectCard = makeCard();
    auto* detectLayout = new QVBoxLayout(detectCard);
    detectLayout->setContentsMargins(18, 14, 18, 14);
    detectLayout->setSpacing(4);
    detectCheck_->setAccessibleDescription(tr("Runs your robot code once to find its motors and sensors"));
    detectLayout->addWidget(detectCheck_);
    detectLayout->addWidget(detectHint_);
    root->addWidget(detectCard);
    root->addStretch(1);

    connect(detectCheck_, &QCheckBox::toggled, &model_, &WizardViewModel::setDetectDevicesAfter);
    connect(&model_, &WizardViewModel::summaryChanged, this, &SummaryPage::refresh);
    refresh();
}

void SummaryPage::refresh() {
    const ProjectInspection& in = model_.inspection();
    const RobotBasics& b = model_.basics();
    const UnitFormatter u = model_.units();

    projectValue_->setText(in.usable() ? QStringLiteral("%1\n%2").arg(in.projectName, in.path) : tr("No project selected"));

    const ToolchainReport& report = model_.toolchain();
    if (!model_.toolchainChecked()) {
        gameValue_->setText(tr("Not checked yet"));
        toolchainValue_->setText(tr("Not checked yet"));
    } else {
        gameValue_->setText(report.coreOnly() ? tr("Core-only mode (no game module for %1)").arg(in.wpilibYear > 0 ? QString::number(in.wpilibYear) : tr("this season"))
                                              : report.gamePluginName);
        toolchainValue_->setText(report.canRunRobot() ? tr("Ready to launch the robot")
                                                      : tr("Something is missing: fix it before running (see step 2)"));
    }

    const QString kind = b.isSwerve() ? tr("swerve") : b.isMecanum() ? tr("mecanum") : tr("tank");
    robotValue_->setText(tr("%1: %2 drivetrain, %3 wheels, %4 × %5, %6")
                             .arg(b.name, kind)
                             .arg(b.wheelCount())
                             .arg(u.formatLength(b.frameLengthM), u.formatLength(b.frameWidthM), u.formatMass(b.massKg)));
    locationValue_->setText(tr("The project, at .wpiemu/robot.json. Commit it to share the setup with your team."));

    const bool possible = model_.discoveryPossible();
    detectCheck_->setEnabled(possible);
    {
        const QSignalBlocker block(detectCheck_);
        detectCheck_->setChecked(possible && model_.detectDevicesAfter());
    }
    detectHint_->setText(possible ? tr("Runs your robot code once so the editor can list its motors, encoders and sensors for you to map.")
                                  : tr("Unavailable until the toolchain check passes. You can detect devices from the editor later."));
}

}  // namespace wpiemu::app
