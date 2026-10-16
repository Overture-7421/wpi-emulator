#include "common/ui_helpers.h"

#include <QHBoxLayout>
#include <QStyle>

namespace wpiemu::app {

QFrame* makeCard(QWidget* parent) {
    auto* card = new QFrame(parent);
    card->setProperty("card", true);
    card->setFrameShape(QFrame::NoFrame);
    return card;
}

QLabel* makeLabel(const QString& text, const char* role, QWidget* parent, bool wrap) {
    auto* label = new QLabel(text, parent);
    label->setProperty("role", QString::fromLatin1(role));
    label->setWordWrap(wrap);
    return label;
}

void repolish(QWidget* widget) {
    widget->style()->unpolish(widget);
    widget->style()->polish(widget);
    widget->update();
}

Banner::Banner(const UiResources& ui, QWidget* parent) : QFrame(parent), ui_(ui), icon_(new QLabel), label_(new QLabel) {
    auto* layout = new QHBoxLayout(this);
    layout->setContentsMargins(12, 9, 12, 9);
    layout->setSpacing(10);
    icon_->setFixedSize(20, 20);
    label_->setWordWrap(true);
    label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(icon_, 0, Qt::AlignTop);
    layout->addWidget(label_, 1);
    connect(&ui_, &UiResources::themeChanged, this, &Banner::refreshIcon);
    hide();
}

void Banner::setMessage(BannerKind kind, const QString& text) {
    kind_ = kind;
    label_->setText(text);
    const char* style = kind == BannerKind::Warning ? "warning" : kind == BannerKind::Error ? "error"
                        : kind == BannerKind::Success ? "success" : "info";
    setProperty("banner", QString::fromLatin1(style));
    repolish(this);
    refreshIcon();
    setVisible(!text.isEmpty());
}

void Banner::clear() {
    label_->clear();
    hide();
}

void Banner::refreshIcon() {
    const bool warn = kind_ == BannerKind::Warning, err = kind_ == BannerKind::Error;
    const QString name = err ? QStringLiteral("x-circle") : warn ? QStringLiteral("alert-triangle")
                         : kind_ == BannerKind::Success ? QStringLiteral("check-circle") : QStringLiteral("info");
    const IconTone tone = err ? IconTone::Error : warn ? IconTone::Warning
                          : kind_ == BannerKind::Success ? IconTone::Success : IconTone::Accent;
    icon_->setPixmap(ui_.icon(name, tone).pixmap(20, 20));
}

}  // namespace wpiemu::app
