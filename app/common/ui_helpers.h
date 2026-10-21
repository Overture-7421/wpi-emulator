#pragma once

#include <QFrame>
#include <QLabel>
#include <QString>

#include "common/ui_resources.h"

class QVBoxLayout;

namespace wpiemu::app {

// Small view helpers shared by the wizard and the shell.

// A rounded panel ("card" style class).
QFrame* makeCard(QWidget* parent = nullptr);
// Label styled by role: title, subtitle, section, muted, error, warning, success.
QLabel* makeLabel(const QString& text, const char* role, QWidget* parent = nullptr, bool wrap = false);
// Selectable, word-wrapped label for a read-only value; its text is set later.
QLabel* makeValueLabel();
// Re-applies the style sheet after a dynamic property changed.
void repolish(QWidget* widget);

enum class BannerKind { Info, Success, Warning, Error };

// Icon + message strip used for validation feedback. Hidden while the text is empty.
class Banner : public QFrame {
    Q_OBJECT
public:
    Banner(const UiResources& ui, QWidget* parent = nullptr);

    void setMessage(BannerKind kind, const QString& text);
    void clear();
    QString text() const { return label_->text(); }
    BannerKind kind() const { return kind_; }

private:
    void refreshIcon();

    const UiResources& ui_;
    QLabel* icon_;
    QLabel* label_;
    BannerKind kind_ = BannerKind::Info;
};

}  // namespace wpiemu::app
