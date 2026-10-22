#pragma once

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QLabel>
#include <QLineEdit>
#include <QSlider>

#include "common/app_settings.h"
#include "common/system_probe.h"
#include "common/toolchain_provisioner.h"
#include "common/ui_resources.h"

namespace wpiemu::app {

// Preferences: appearance, toolchain locations, simulation rate and physics threads. Edits a copy and only writes
// to AppSettings on OK.
class SettingsDialog : public QDialog {
    Q_OBJECT
public:
    SettingsDialog(AppSettings& settings, const ISystemProbe& probe, const UiResources& ui, QWidget* parent = nullptr,
                   ToolchainProvisioner* java = nullptr);

    AppSettingsData collect() const;
    // Problems in the current input; non-empty disables OK.
    QStringList problems() const;

    QComboBox* themeCombo() const { return theme_; }
    QComboBox* rateCombo() const { return rate_; }
    QComboBox* threadsCombo() const { return threads_; }
    QLineEdit* jdkEdit() const { return jdk_; }
    QCheckBox* soundCheck() const { return sound_; }
    QCheckBox* updatesCheck() const { return updates_; }
    QSlider* volumeSlider() const { return volume_; }
    QDialogButtonBox* buttons() const { return buttons_; }
    QPushButton* removeJavaButton() const { return removeJava_; }

public slots:
    void accept() override;

private:
    void load(const AppSettingsData& data);
    void validate();
    void refreshJava();
    QWidget* pathRow(QLineEdit* edit, const QString& dialogTitle);

    AppSettings& settings_;
    const ISystemProbe& probe_;
    ToolchainProvisioner* java_;
    QLabel* javaStatus_ = nullptr;
    QPushButton* removeJava_ = nullptr;
    QComboBox* theme_;
    QComboBox* units_;
    QComboBox* rate_;
    QComboBox* threads_;
    QLineEdit* wpilib_;
    QLineEdit* jdk_;
    QCheckBox* reopen_;
    QCheckBox* sound_;
    QCheckBox* updates_;
    QSlider* volume_;
    QLabel* problemLabel_;
    QDialogButtonBox* buttons_;
};

}  // namespace wpiemu::app
