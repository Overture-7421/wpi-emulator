#pragma once

#include <QString>
#include <QWidget>

namespace wpiemu::app {

// One step of the setup wizard. Pages are views only: they read from and write to the
// WizardViewModel they are constructed with.
class WizardPage : public QWidget {
    Q_OBJECT
public:
    using QWidget::QWidget;

    virtual QString title() const = 0;
    virtual QString subtitle() const = 0;
    // Called each time the page becomes the current one.
    virtual void onEnter() {}
    // Widget that should receive keyboard focus first (may be null).
    virtual QWidget* initialFocus() const { return nullptr; }
};

}  // namespace wpiemu::app
