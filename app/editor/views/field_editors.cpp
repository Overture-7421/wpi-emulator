#include "views/field_editors.h"

#include <QColorDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <cmath>

#include "model/quat_utils.h"
#include "model/tr.h"
#include "wpiemu/sdk/config/value_codec.h"

namespace wpiemu::app::editor {

QuietSpinBox::QuietSpinBox(QWidget* parent) : QDoubleSpinBox(parent) {
    setFocusPolicy(Qt::StrongFocus);
    setKeyboardTracking(true);
    setButtonSymbols(QAbstractSpinBox::NoButtons);
    setAlignment(Qt::AlignRight);
}

void QuietSpinBox::wheelEvent(QWheelEvent* event) {
    if (!hasFocus()) event->ignore();
    else QDoubleSpinBox::wheelEvent(event);
}

// ---- bool ---------------------------------------------------------------------------------------
BoolEditor::BoolEditor(QWidget* parent) : FieldEditor(parent), box_(new QCheckBox(this)) {
    auto* l = new QHBoxLayout(this);
    l->setContentsMargins(0, 0, 0, 0);
    l->addWidget(box_);
    l->addStretch();
    connect(box_, &QCheckBox::toggled, this, [this](bool on) { Q_EMIT edited(sdk::Value(on)); });
}

void BoolEditor::setValue(const sdk::Value& value) {
    const QSignalBlocker block(box_);
    box_->setChecked(value.asBool());
}

// ---- number -------------------------------------------------------------------------------------
NumberEditor::NumberEditor(const sdk::PropertySpec& spec, UnitSystem units, QWidget* parent)
    : FieldEditor(parent), spec_(spec), units_(units), spin_(new QuietSpinBox(this)) {
    auto* l = new QHBoxLayout(this);
    l->setContentsMargins(0, 0, 0, 0);
    l->addWidget(spin_);

    const bool isInt = spec.type == sdk::PropertyType::Int;
    const UnitDisplay ud = unitDisplay(spec.unit, units);
    spin_->setSuffix(QString::fromUtf8(ud.suffix));
    double step = spec.step > 0.0 ? spec.step * ud.factor : ud.step;
    int decimals = ud.decimals;
    if (isInt) {
        decimals = 0;
        step = std::max(1.0, std::round(step));
    } else {
        while (decimals < 6 && step < std::pow(10.0, -decimals) * 0.999) ++decimals;
    }
    spin_->setDecimals(decimals);
    spin_->setSingleStep(step);
    spin_->setRange(spec.min ? *spec.min * ud.factor : -1e6, spec.max ? *spec.max * ud.factor : 1e6);
    spin_->setMinimumWidth(70);
    if (spec_.description.size() > 0) spin_->setToolTip(QString::fromStdString(spec_.description));

    connect(spin_, &QDoubleSpinBox::valueChanged, this, [this](double) { Q_EMIT edited(value()); });
}

void NumberEditor::setValue(const sdk::Value& value) {
    const QSignalBlocker block(spin_);
    spin_->setValue(toDisplay(value.asDouble(), spec_.unit, units_));
}

sdk::Value NumberEditor::value() const {
    const double si = fromDisplay(spin_->value(), spec_.unit, units_);
    if (spec_.type == sdk::PropertyType::Int) return sdk::Value(static_cast<std::int64_t>(std::llround(si)));
    return sdk::Value(si);
}

// ---- text ---------------------------------------------------------------------------------------
TextEditor::TextEditor(QWidget* parent) : FieldEditor(parent), edit_(new QLineEdit(this)) {
    auto* l = new QHBoxLayout(this);
    l->setContentsMargins(0, 0, 0, 0);
    l->addWidget(edit_);
    connect(edit_, &QLineEdit::editingFinished, this, [this] { Q_EMIT edited(value()); });
}

void TextEditor::setValue(const sdk::Value& value) {
    const QSignalBlocker block(edit_);
    if (!edit_->hasFocus()) edit_->setText(QString::fromStdString(value.asString()));
}

// ---- enum ---------------------------------------------------------------------------------------
ChoiceEditor::ChoiceEditor(const sdk::PropertySpec& spec, QWidget* parent) : FieldEditor(parent), combo_(new QComboBox(this)) {
    auto* l = new QHBoxLayout(this);
    l->setContentsMargins(0, 0, 0, 0);
    l->addWidget(combo_);
    combo_->setFocusPolicy(Qt::StrongFocus);
    for (const auto& o : spec.options) combo_->addItem(QString::fromStdString(o.label.empty() ? o.value : o.label), QString::fromStdString(o.value));
    connect(combo_, &QComboBox::activated, this, [this](int) { Q_EMIT edited(value()); });
}

void ChoiceEditor::setValue(const sdk::Value& value) {
    const QSignalBlocker block(combo_);
    const QString v = QString::fromStdString(value.asString());
    int idx = combo_->findData(v);
    if (idx < 0) {
        combo_->addItem(ETR("(unknown: %1)").arg(v), v);
        idx = combo_->count() - 1;
    }
    combo_->setCurrentIndex(idx);
}

sdk::Value ChoiceEditor::value() const { return sdk::Value(combo_->currentData().toString().toStdString()); }

// ---- colour -------------------------------------------------------------------------------------
ColorEditor::ColorEditor(QWidget* parent) : FieldEditor(parent), button_(new QPushButton(this)) {
    auto* l = new QHBoxLayout(this);
    l->setContentsMargins(0, 0, 0, 0);
    l->addWidget(button_);
    connect(button_, &QPushButton::clicked, this, [this] { pick(); });
}

void ColorEditor::paint() {
    const QColor c(color_);
    button_->setText(color_);
    button_->setStyleSheet(QStringLiteral("QPushButton { text-align: left; padding: 3px 8px; }"));
    QPixmap pm(16, 16);
    pm.fill(c.isValid() ? c : QColor("#808080"));
    button_->setIcon(QIcon(pm));
}

void ColorEditor::setValue(const sdk::Value& value) {
    color_ = QString::fromStdString(value.asString("#808080"));
    paint();
}

void ColorEditor::pick() {
    const QColor chosen = QColorDialog::getColor(QColor(color_), this, ETR("Pick a colour"));
    if (!chosen.isValid()) return;
    color_ = chosen.name();
    paint();
    Q_EMIT edited(value());
}

// ---- vectors ------------------------------------------------------------------------------------
VectorEditor::VectorEditor(sdk::PropertyUnit unit, UnitSystem units, const QStringList& labels, QWidget* parent)
    : FieldEditor(parent), unit_(unit), units_(units) {
    auto* l = new QHBoxLayout(this);
    l->setContentsMargins(0, 0, 0, 0);
    l->setSpacing(4);
    const UnitDisplay ud = unitDisplay(unit, units);
    for (int i = 0; i < 3; ++i) {
        auto* tag = new QLabel(labels.value(i), this);
        tag->setEnabled(false);
        spins_[static_cast<std::size_t>(i)] = new QuietSpinBox(this);
        auto* spin = spins_[static_cast<std::size_t>(i)];
        spin->setDecimals(ud.decimals);
        spin->setSingleStep(ud.step);
        spin->setRange(-1e6, 1e6);
        spin->setMinimumWidth(44);
        spin->setSuffix(QString::fromUtf8(ud.suffix));
        l->addWidget(tag);
        l->addWidget(spin, 1);
        connect(spin, &QDoubleSpinBox::valueChanged, this, [this](double) {
            rotDirty_ = true;
            Q_EMIT edited(value());
        });
    }
}



void VectorEditor::setValue(const sdk::Value& value) {
    std::array<QSignalBlocker, 3> blockers{QSignalBlocker(spins_[0]), QSignalBlocker(spins_[1]), QSignalBlocker(spins_[2])};
    if (rotation_) {
        quat_ = sdk::Quat{};
        sdk::fromValue(value, quat_);
        rotDirty_ = false;
        const Euler e = toEuler(quat_);
        spins_[0]->setValue(radToDegF(e.rollRad));
        spins_[1]->setValue(radToDegF(e.pitchRad));
        spins_[2]->setValue(radToDegF(e.yawRad));
        return;
    }
    sdk::Vec3 v;
    sdk::fromValue(value, v);
    spins_[0]->setValue(toDisplay(static_cast<double>(v.x), unit_, units_));
    spins_[1]->setValue(toDisplay(static_cast<double>(v.y), unit_, units_));
    spins_[2]->setValue(toDisplay(static_cast<double>(v.z), unit_, units_));
}

sdk::Value VectorEditor::value() const {
    if (rotation_) {
        if (!rotDirty_) return sdk::toValue(quat_);
        const Euler e{degToRadF(static_cast<float>(spins_[0]->value())), degToRadF(static_cast<float>(spins_[1]->value())),
                      degToRadF(static_cast<float>(spins_[2]->value()))};
        return sdk::toValue(fromEuler(e));
    }
    return sdk::toValue(sdk::Vec3{static_cast<float>(fromDisplay(spins_[0]->value(), unit_, units_)),
                                  static_cast<float>(fromDisplay(spins_[1]->value(), unit_, units_)),
                                  static_cast<float>(fromDisplay(spins_[2]->value(), unit_, units_))});
}

CompositeEditor::CompositeEditor(sdk::PropertyType type, sdk::PropertyUnit unit, UnitSystem units, QWidget* parent)
    : FieldEditor(parent), type_(type) {
    auto* l = new QVBoxLayout(this);
    l->setContentsMargins(0, 0, 0, 0);
    l->setSpacing(2);
    const QStringList xyz{QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z")};
    auto caption = [&](const QString& text) {
        auto* c = new QLabel(text, this);
        c->setEnabled(false);
        QFont f = c->font();
        f.setPointSizeF(f.pointSizeF() * 0.85);
        c->setFont(f);
        l->addWidget(c);
    };
    caption(ETR("Position"));
    position_ = new VectorEditor(unit == sdk::PropertyUnit::None ? sdk::PropertyUnit::Meters : unit, units, xyz, this);
    l->addWidget(position_);
    caption(ETR("Rotation (roll, pitch, yaw)"));
    rotation_ = new VectorEditor(sdk::PropertyUnit::Radians, units, {QStringLiteral("R"), QStringLiteral("P"), QStringLiteral("Y")}, this);
    rotation_->setRotationMode(true);
    l->addWidget(rotation_);
    connect(position_, &FieldEditor::edited, this, [this] { emitEdited(); });
    connect(rotation_, &FieldEditor::edited, this, [this] { emitEdited(); });
    if (type == sdk::PropertyType::Box3) {
        caption(ETR("Size"));
        size_ = new VectorEditor(unit == sdk::PropertyUnit::None ? sdk::PropertyUnit::Meters : unit, units, xyz, this);
        l->addWidget(size_);
        connect(size_, &FieldEditor::edited, this, [this] { emitEdited(); });
    }
}

void CompositeEditor::setValue(const sdk::Value& value) {
    const sdk::Value* pose = type_ == sdk::PropertyType::Box3 ? value.find("pose") : &value;
    if (pose != nullptr) {
        position_->setValue(pose->get("position"));
        rotation_->setValue(pose->get("rotation"));
    }
    if (size_ != nullptr) size_->setValue(value.get("size"));
}

sdk::Value CompositeEditor::value() const {
    sdk::Value pose = sdk::Value::makeObject();
    pose["position"] = position_->value();
    pose["rotation"] = rotation_->value();
    if (type_ != sdk::PropertyType::Box3) return pose;
    sdk::Value box = sdk::Value::makeObject();
    box["pose"] = pose;
    box["size"] = size_->value();
    return box;
}

FieldEditor* createFieldEditor(const sdk::PropertySpec& spec, UnitSystem units, QWidget* parent) {
    using T = sdk::PropertyType;
    FieldEditor* editor = nullptr;
    switch (spec.type) {
        case T::Bool: editor = new BoolEditor(parent); break;
        case T::Int:
        case T::Double: editor = new NumberEditor(spec, units, parent); break;
        case T::String: editor = new TextEditor(parent); break;
        case T::Enum: editor = new ChoiceEditor(spec, parent); break;
        case T::Color: editor = new ColorEditor(parent); break;
        case T::Vec3: editor = new VectorEditor(spec.unit == sdk::PropertyUnit::None ? sdk::PropertyUnit::None : spec.unit, units,
                                                {QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z")}, parent); break;
        case T::Pose3:
        case T::Box3: editor = new CompositeEditor(spec.type, spec.unit, units, parent); break;
    }
    if (editor != nullptr && !spec.description.empty()) editor->setToolTip(QString::fromStdString(spec.description));
    return editor;
}

}  // namespace wpiemu::app::editor
