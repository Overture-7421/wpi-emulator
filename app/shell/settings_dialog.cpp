#include "shell/settings_dialog.h"

#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTabWidget>
#include <QVBoxLayout>

#include "common/ui_helpers.h"

namespace wpiemu::app {

SettingsDialog::SettingsDialog(AppSettings& settings, const ISystemProbe& probe, const UiResources& ui, QWidget* parent)
    : QDialog(parent),
      settings_(settings),
      probe_(probe),
      theme_(new QComboBox),
      units_(new QComboBox),
      rate_(new QComboBox),
      wpilib_(new QLineEdit),
      jdk_(new QLineEdit),
      reopen_(new QCheckBox(tr("Reopen the last project when the app starts"))),
      problemLabel_(makeLabel({}, "error", nullptr, true)),
      buttons_(new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel)) {
    setWindowTitle(tr("Settings"));
    setModal(true);
    setMinimumWidth(560);

    theme_->addItem(tr("Follow system"), static_cast<int>(ThemeMode::System));
    theme_->addItem(tr("Light"), static_cast<int>(ThemeMode::Light));
    theme_->addItem(tr("Dark"), static_cast<int>(ThemeMode::Dark));
    units_->addItem(tr("Imperial (in, lb)"), static_cast<int>(UnitSystem::Imperial));
    units_->addItem(tr("Metric (cm, kg)"), static_cast<int>(UnitSystem::Metric));
    for (const int hz : kPhysicsRatesHz)
        rate_->addItem(hz == kDefaultPhysicsRateHz ? tr("%1 Hz (default)").arg(hz) : tr("%1 Hz").arg(hz), hz);

    auto* tabs = new QTabWidget;

    auto* appearance = new QWidget;
    auto* appearanceForm = new QFormLayout(appearance);
    appearanceForm->setContentsMargins(18, 18, 18, 18);
    appearanceForm->setVerticalSpacing(12);
    appearanceForm->addRow(tr("Theme"), theme_);
    appearanceForm->addRow(tr("Units in dialogs"), units_);
    tabs->addTab(appearance, ui.icon(QStringLiteral("sun")), tr("Appearance"));

    auto* paths = new QWidget;
    auto* pathsLayout = new QVBoxLayout(paths);
    pathsLayout->setContentsMargins(18, 18, 18, 18);
    auto* pathsForm = new QFormLayout;
    pathsForm->setVerticalSpacing(12);
    wpilib_->setPlaceholderText(tr("Detected automatically"));
    jdk_->setPlaceholderText(tr("Detected automatically (WPILib's JDK, then JAVA_HOME)"));
    pathsForm->addRow(tr("WPILib folder"), pathRow(wpilib_, tr("Select the WPILib installation folder")));
    pathsForm->addRow(tr("JDK folder"), pathRow(jdk_, tr("Select the JDK folder")));
    pathsLayout->addLayout(pathsForm);
    pathsLayout->addWidget(makeLabel(tr("Leave a field empty to detect it automatically. Changes apply the next time the toolchain is checked or the robot is launched."), "muted", nullptr, true));
    pathsLayout->addStretch(1);
    tabs->addTab(paths, ui.icon(QStringLiteral("folder-open")), tr("Toolchain"));

    auto* simulation = new QWidget;
    auto* simulationForm = new QFormLayout(simulation);
    simulationForm->setContentsMargins(18, 18, 18, 18);
    simulationForm->setVerticalSpacing(12);
    simulationForm->addRow(tr("Physics rate"), rate_);
    simulationForm->addRow(makeLabel(tr("Higher rates are more accurate but need more CPU. Applies to the next robot run."), "muted", nullptr, true));
    tabs->addTab(simulation, ui.icon(QStringLiteral("run")), tr("Simulation"));

    auto* general = new QWidget;
    auto* generalLayout = new QVBoxLayout(general);
    generalLayout->setContentsMargins(18, 18, 18, 18);
    generalLayout->addWidget(reopen_);
    generalLayout->addStretch(1);
    tabs->addTab(general, ui.icon(QStringLiteral("settings")), tr("General"));

    auto* root = new QVBoxLayout(this);
    root->addWidget(tabs);
    root->addWidget(problemLabel_);
    root->addWidget(buttons_);

    connect(buttons_, &QDialogButtonBox::accepted, this, &SettingsDialog::accept);
    connect(buttons_, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(wpilib_, &QLineEdit::textChanged, this, &SettingsDialog::validate);
    connect(jdk_, &QLineEdit::textChanged, this, &SettingsDialog::validate);

    load(settings_.data());
    validate();
}

QWidget* SettingsDialog::pathRow(QLineEdit* edit, const QString& dialogTitle) {
    auto* w = new QWidget;
    auto* l = new QHBoxLayout(w);
    l->setContentsMargins(0, 0, 0, 0);
    edit->setClearButtonEnabled(true);
    auto* browse = new QPushButton(tr("Browse…"));
    l->addWidget(edit, 1);
    l->addWidget(browse);
    connect(browse, &QPushButton::clicked, this, [this, edit, dialogTitle] {
        const QString dir = QFileDialog::getExistingDirectory(this, dialogTitle, edit->text());
        if (!dir.isEmpty()) edit->setText(QDir::toNativeSeparators(dir));
    });
    return w;
}

void SettingsDialog::load(const AppSettingsData& d) {
    theme_->setCurrentIndex(theme_->findData(static_cast<int>(d.theme)));
    units_->setCurrentIndex(units_->findData(static_cast<int>(d.units)));
    rate_->setCurrentIndex(rate_->findData(d.physicsRateHz));
    wpilib_->setText(d.wpilibHome);
    jdk_->setText(d.jdkHome);
    reopen_->setChecked(d.reopenLastProject);
}

AppSettingsData SettingsDialog::collect() const {
    AppSettingsData d = settings_.data();
    d.theme = static_cast<ThemeMode>(theme_->currentData().toInt());
    d.units = static_cast<UnitSystem>(units_->currentData().toInt());
    d.physicsRateHz = rate_->currentData().toInt();
    d.wpilibHome = wpilib_->text().trimmed();
    d.jdkHome = jdk_->text().trimmed();
    d.reopenLastProject = reopen_->isChecked();
    return d;
}

QStringList SettingsDialog::problems() const {
    QStringList out;
    const auto check = [&](const QLineEdit* edit, const QString& what) {
        const QString p = edit->text().trimmed();
        if (!p.isEmpty() && !probe_.dirExists(p)) out << tr("The %1 does not exist: %2").arg(what, p);
    };
    check(wpilib_, tr("WPILib folder"));
    check(jdk_, tr("JDK folder"));
    return out;
}

void SettingsDialog::validate() {
    const QStringList issues = problems();
    problemLabel_->setText(issues.join(QLatin1Char('\n')));
    problemLabel_->setVisible(!issues.isEmpty());
    buttons_->button(QDialogButtonBox::Ok)->setEnabled(issues.isEmpty());
}

void SettingsDialog::accept() {
    if (!problems().isEmpty()) return;
    settings_.update(collect());
    QDialog::accept();
}

}  // namespace wpiemu::app
