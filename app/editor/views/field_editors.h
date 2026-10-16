#pragma once

// Typed value editors for the generic property form. Each edits one SDK `Value` (always SI) and shows
// it in the user's units. They only emit `edited` for USER changes, never when the form pushes a value.

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <QWidget>

#include <array>
#include <memory>

#include "model/units.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/config/property_schema.h"

namespace wpiemu::app::editor {

// Spin box that does not hijack the mouse wheel while the user scrolls the inspector.
class QuietSpinBox : public QDoubleSpinBox {
    Q_OBJECT
public:
    explicit QuietSpinBox(QWidget* parent = nullptr);

protected:
    void wheelEvent(QWheelEvent* event) override;
};

class FieldEditor : public QWidget {
    Q_OBJECT
public:
    explicit FieldEditor(QWidget* parent = nullptr) : QWidget(parent) {}
    virtual void setValue(const sdk::Value& value) = 0;
    virtual sdk::Value value() const = 0;
    virtual QWidget* focusTarget() { return this; }

Q_SIGNALS:
    void edited(const sdk::Value& value);
};

// Creates the right editor for `spec`, converting units for `units`.
FieldEditor* createFieldEditor(const sdk::PropertySpec& spec, UnitSystem units, QWidget* parent = nullptr);

class BoolEditor final : public FieldEditor {
    Q_OBJECT
public:
    explicit BoolEditor(QWidget* parent = nullptr);
    void setValue(const sdk::Value& value) override;
    sdk::Value value() const override { return sdk::Value(box_->isChecked()); }
    QWidget* focusTarget() override { return box_; }

private:
    QCheckBox* box_;
};

class NumberEditor final : public FieldEditor {
    Q_OBJECT
public:
    NumberEditor(const sdk::PropertySpec& spec, UnitSystem units, QWidget* parent = nullptr);
    void setValue(const sdk::Value& value) override;
    sdk::Value value() const override;
    QWidget* focusTarget() override { return spin_; }

private:
    sdk::PropertySpec spec_;
    UnitSystem units_;
    QuietSpinBox* spin_;
};

class TextEditor final : public FieldEditor {
    Q_OBJECT
public:
    explicit TextEditor(QWidget* parent = nullptr);
    void setValue(const sdk::Value& value) override;
    sdk::Value value() const override { return sdk::Value(edit_->text().toStdString()); }
    QWidget* focusTarget() override { return edit_; }

private:
    QLineEdit* edit_;
};

class ChoiceEditor final : public FieldEditor {
    Q_OBJECT
public:
    ChoiceEditor(const sdk::PropertySpec& spec, QWidget* parent = nullptr);
    void setValue(const sdk::Value& value) override;
    sdk::Value value() const override;
    QWidget* focusTarget() override { return combo_; }

private:
    QComboBox* combo_;
};

class ColorEditor final : public FieldEditor {
    Q_OBJECT
public:
    explicit ColorEditor(QWidget* parent = nullptr);
    void setValue(const sdk::Value& value) override;
    sdk::Value value() const override { return sdk::Value(color_.toStdString()); }
    QWidget* focusTarget() override { return button_; }

private:
    void pick();
    void paint();
    QPushButton* button_;
    QString color_ = QStringLiteral("#808080");
};

// n spin boxes in a row (Vec3, or position / rotation / size parts of Pose3 and Box3).
class VectorEditor : public FieldEditor {
    Q_OBJECT
public:
    // `unit` applies to every component; rotation rows pass PropertyUnit::Radians and show degrees.
    VectorEditor(sdk::PropertyUnit unit, UnitSystem units, const QStringList& labels, QWidget* parent = nullptr);
    void setValue(const sdk::Value& value) override;       // {"x","y","z"}
    sdk::Value value() const override;
    QWidget* focusTarget() override { return spins_.front(); }
    void setRotationMode(bool on) { rotation_ = on; }      // value is a quaternion; rows show roll/pitch/yaw

private:
    sdk::PropertyUnit unit_;
    UnitSystem units_;
    bool rotation_ = false;
    sdk::Quat quat_;           // rotation rows keep the original quaternion until the user edits a component
    bool rotDirty_ = false;
    std::array<QuietSpinBox*, 3> spins_{};
};

// Pose3 {"position","rotation"} and Box3 {"pose","size"}: stacked rows with captions.
class CompositeEditor final : public FieldEditor {
    Q_OBJECT
public:
    CompositeEditor(sdk::PropertyType type, sdk::PropertyUnit unit, UnitSystem units, QWidget* parent = nullptr);
    void setValue(const sdk::Value& value) override;
    sdk::Value value() const override;
    QWidget* focusTarget() override { return position_->focusTarget(); }

private:
    void emitEdited() { Q_EMIT edited(value()); }
    sdk::PropertyType type_;
    VectorEditor* position_ = nullptr;
    VectorEditor* rotation_ = nullptr;
    VectorEditor* size_ = nullptr;
};

}  // namespace wpiemu::app::editor
