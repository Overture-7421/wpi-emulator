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

SettingsDialog::SettingsDialog(AppSettings& settings, const ISystemProbe& probe, const UiResources& ui, QWidget* parent, ToolchainProvisioner* java)
    : QDialog(parent),
      settings_(settings),
      probe_(probe),
      java_(java),
      theme_(new QComboBox),
      units_(new QComboBox),
      rate_(new QComboBox),
      threads_(new QComboBox),
      wpilib_(new QLineEdit),
      jdk_(new QLineEdit),
      reopen_(new QCheckBox(tr("Reopen the last project when the app starts"))),
      sound_(new QCheckBox(tr("Play match sound effects during Practice"))),
      updates_(new QCheckBox(tr("Check for updates when the app starts"))),
      volume_(new QSlider(Qt::Horizontal)),
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
    for (const int n : kPhysicsThreadChoices)
        threads_->addItem(n == 0 ? tr("Auto (recommended)") : QString::number(n), n);

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
    jdk_->setPlaceholderText(tr("Detected automatically (WPILib's JDK, then JAVA_HOME, then PATH)"));
    pathsForm->addRow(tr("WPILib folder"), pathRow(wpilib_, tr("Select the WPILib installation folder")));
    pathsForm->addRow(tr("JDK folder"), pathRow(jdk_, tr("Select the JDK folder")));
    pathsLayout->addLayout(pathsForm);
    pathsLayout->addWidget(makeLabel(tr("Leave a field empty to detect it automatically. Changes apply the next time the toolchain is checked or the robot is launched."), "muted", nullptr, true));
    if (java_ != nullptr) {
        // The emulator's own Java is only used when no other Java can run the project; it is downloaded from the setup's Toolchain step.
        auto* javaRow = new QHBoxLayout;
        javaStatus_ = makeLabel({}, "muted", nullptr, true);
        removeJava_ = new QPushButton(tr("Remove"));
        javaRow->addWidget(javaStatus_, 1);
        javaRow->addWidget(removeJava_);
        pathsLayout->addSpacing(12);
        pathsLayout->addWidget(makeLabel(tr("WPI Emulator's own Java"), "section"));
        pathsLayout->addLayout(javaRow);
        connect(removeJava_, &QPushButton::clicked, this, [this] {
            java_->remove();
            refreshJava();
        });
        refreshJava();
    }
    pathsLayout->addStretch(1);
    tabs->addTab(paths, ui.icon(QStringLiteral("folder-open")), tr("Toolchain"));

    auto* simulation = new QWidget;
    auto* simulationForm = new QFormLayout(simulation);
    simulationForm->setContentsMargins(18, 18, 18, 18);
    simulationForm->setVerticalSpacing(12);
    simulationForm->addRow(tr("Physics rate"), rate_);
    simulationForm->addRow(makeLabel(tr("Higher rates are more accurate but need more CPU. Applies to the next robot run."), "muted", nullptr, true));
    simulationForm->addRow(tr("Physics threads"), threads_);
    simulationForm->addRow(makeLabel(tr("Threads used to compute physics. Auto picks up to 4 from this computer's cores. More threads help scenes with many game pieces and high physics rates; results are identical for every choice. Applies the next time the robot is launched."), "muted", nullptr, true));
    tabs->addTab(simulation, ui.icon(QStringLiteral("run")), tr("Simulation"));

    auto* general = new QWidget;
    auto* generalLayout = new QVBoxLayout(general);
    generalLayout->setContentsMargins(18, 18, 18, 18);
    generalLayout->addWidget(reopen_);
    generalLayout->addWidget(updates_);
    generalLayout->addWidget(makeLabel(tr("Only asks github.com whether a newer release exists; nothing about you or your projects is sent."), "muted", nullptr, true));
    generalLayout->addSpacing(12);
    generalLayout->addWidget(sound_);
    auto* volumeRow = new QFormLayout;
    volume_->setRange(0, 100);
    volume_->setPageStep(10);
    volumeRow->addRow(tr("Volume"), volume_);
    generalLayout->addLayout(volumeRow);
    generalLayout->addWidget(makeLabel(tr("A Practice match plays the field's start, end and warning sounds, and the foghorn when you E-Stop."), "muted", nullptr, true));
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
    connect(sound_, &QCheckBox::toggled, volume_, &QWidget::setEnabled);

    load(settings_.data());
    validate();
}

void SettingsDialog::refreshJava() {
    if (java_ == nullptr) return;
    const qint64 bytes = java_->installedBytes();
    removeJava_->setVisible(bytes > 0);
    removeJava_->setEnabled(!java_->busy());
    javaStatus_->setText(bytes > 0 ? tr("Downloaded (%1 MB) in %2. Used only when no other Java can run the project.")
                                         .arg((bytes + 500000) / 1000000)
                                         .arg(QDir::toNativeSeparators(java_->toolchainsDir()))
                                   : tr("Not downloaded. If no Java that can run a project is found, the setup offers to download one."));
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
    threads_->setCurrentIndex(threads_->findData(d.physicsThreads));
    wpilib_->setText(d.wpilibHome);
    jdk_->setText(d.jdkHome);
    reopen_->setChecked(d.reopenLastProject);
    sound_->setChecked(d.soundEffects);
    updates_->setChecked(d.checkForUpdates);
    volume_->setValue(d.soundVolumePercent);
    volume_->setEnabled(d.soundEffects);
}

AppSettingsData SettingsDialog::collect() const {
    AppSettingsData d = settings_.data();
    d.theme = static_cast<ThemeMode>(theme_->currentData().toInt());
    d.units = static_cast<UnitSystem>(units_->currentData().toInt());
    d.physicsRateHz = rate_->currentData().toInt();
    d.physicsThreads = threads_->currentData().toInt();
    d.wpilibHome = wpilib_->text().trimmed();
    d.jdkHome = jdk_->text().trimmed();
    d.reopenLastProject = reopen_->isChecked();
    d.soundEffects = sound_->isChecked();
    d.checkForUpdates = updates_->isChecked();
    d.soundVolumePercent = volume_->value();
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
