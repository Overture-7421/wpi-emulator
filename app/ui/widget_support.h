#pragma once

// Small widget recipes the editor, run view and shell repeat. Qt Widgets only.

#include <QString>
#include <QWidget>

class QLabel;

namespace wpiemu::app::ui {

// A word-wrapped label in the disabled colour, for hints and secondary text.
QLabel* mutedLabel(const QString& text, QWidget* parent);

// A label a little smaller than body text, in the disabled colour.
QLabel* captionLabel(const QString& text, QWidget* parent);

// An upper-cased, bold, letter-spaced caption in the disabled colour for a group of controls; the caller sets margins.
QLabel* headingLabel(const QString& text, QWidget* parent);

// How long flashWidget() highlights a widget.
inline constexpr int kFlashMs = 1400;

// Applies `styleSheet` to `widget` for kFlashMs, then clears its style sheet; a way to point out a row or field.
void flashWidget(QWidget* widget, const QString& styleSheet);

// Makes the mouse wheel scroll the enclosing form instead of editing `input`. A combo box never reacts to the wheel;
// any other input (spin box, slider) reacts only after the user clicked or tabbed into it.
void setQuietWheel(QWidget* input);

// setQuietWheel() as an expression: `ui::quietWheel(new QComboBox(this))`.
template <class Input>
Input* quietWheel(Input* input) {
    setQuietWheel(input);
    return input;
}

}  // namespace wpiemu::app::ui
