#pragma once

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QLabel>
#include <QLineEdit>

#include "common/app_settings.h"
#include "common/system_probe.h"
#include "common/ui_resources.h"

namespace wpiemu::app {

// Preferences: appearance, toolchain locations, simulation rate. Edits a copy and only writes
// to AppSettings on OK.
class SettingsDialog : public QDialog {
    Q_OBJECT
public:
    SettingsDialog(AppSettings& settings, const ISystemProbe& probe, const UiResources& ui, QWidget* parent = nullptr);

    AppSettingsData collect() const;
    // Problems in the current input; non-empty disables OK.
    QStringList problems() const;

    QComboBox* themeCombo() const { return theme_; }
    QComboBox* rateCombo() const { return rate_; }
    QLineEdit* jdkEdit() const { return jdk_; }
    QDialogButtonBox* buttons() const { return buttons_; }

public slots:
    void accept() override;

private:
    void load(const AppSettingsData& data);
    void validate();
    QWidget* pathRow(QLineEdit* edit, const QString& dialogTitle);

    AppSettings& settings_;
    const ISystemProbe& probe_;
    QComboBox* theme_;
    QComboBox* units_;
    QComboBox* rate_;
    QLineEdit* wpilib_;
    QLineEdit* jdk_;
    QCheckBox* reopen_;
    QLabel* problemLabel_;
    QDialogButtonBox* buttons_;
};

}  // namespace wpiemu::app
