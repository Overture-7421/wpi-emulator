#pragma once

// Transient notifications stacked in the bottom-right corner of the host widget ("New device
// detected: TalonFX 14"). Non-modal, click to dismiss, fade out by themselves.

#include <QFrame>
#include <QWidget>

#include <vector>

namespace wpiemu::app::editor {

class ToastOverlay : public QWidget {
    Q_OBJECT
public:
    explicit ToastOverlay(QWidget* host);

    // severity: 0 info, 1 warning, 2 error
    void showToast(const QString& title, const QString& detail, int severity = 0, int durationMs = 4200);
    int activeCount() const;

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void relayout();

    QWidget* host_;
    std::vector<QFrame*> toasts_;
};

}  // namespace wpiemu::app::editor
