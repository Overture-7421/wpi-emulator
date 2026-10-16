#pragma once

#include <QColor>
#include <QLabel>

#include "run_control.h"

namespace wpiemu::app::runview {

QColor toneColor(Tone tone);

// Rounded pill with a coloured dot: connection / process / real-DS state.
class StatusChip : public QLabel {
    Q_OBJECT
public:
    explicit StatusChip(QWidget* parent = nullptr);
    void setStatus(Tone tone, const QString& text, const QString& tooltip = {});
    Tone tone() const { return tone_; }

private:
    Tone tone_ = Tone::Neutral;
};

}  // namespace wpiemu::app::runview
