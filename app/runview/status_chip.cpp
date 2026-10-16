#include "status_chip.h"

namespace wpiemu::app::runview {

QColor toneColor(Tone tone) {
    switch (tone) {
        case Tone::Success: return QColor(0x2f, 0xb3, 0x6d);
        case Tone::Info: return QColor(0x3b, 0x82, 0xf6);
        case Tone::Warning: return QColor(0xe0, 0xa5, 0x26);
        case Tone::Error: return QColor(0xe5, 0x48, 0x4d);
        case Tone::Neutral: break;
    }
    return QColor(0x8b, 0x95, 0xa5);
}

StatusChip::StatusChip(QWidget* parent) : QLabel(parent) {
    setTextFormat(Qt::RichText);
    setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
    setStatus(Tone::Neutral, {});
}

void StatusChip::setStatus(Tone tone, const QString& text, const QString& tooltip) {
    tone_ = tone;
    const QColor c = toneColor(tone);
    setStyleSheet(QStringLiteral("QLabel { background: rgba(%1,%2,%3,40); border: 1px solid rgba(%1,%2,%3,150);"
                                 " border-radius: 11px; padding: 2px 10px; }")
                      .arg(c.red())
                      .arg(c.green())
                      .arg(c.blue()));
    setText(QStringLiteral("<span style=\"color:%1\">●</span>&nbsp;%2").arg(c.name(), text.toHtmlEscaped()));
    setToolTip(tooltip);
    setAccessibleName(text);
}

}  // namespace wpiemu::app::runview
