#include "views/property_form.h"

#include <QApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QScrollArea>
#include <QTimer>
#include <QVBoxLayout>

#include "model/tr.h"

namespace wpiemu::app::editor {

namespace {

QString qs(const std::string& s) { return QString::fromStdString(s); }

QLabel* subheading(const QString& text, QWidget* parent) {
    auto* l = new QLabel(text.toUpper(), parent);
    QFont f = l->font();
    f.setBold(true);
    f.setPointSizeF(f.pointSizeF() * 0.8);
    f.setLetterSpacing(QFont::PercentageSpacing, 108);
    l->setFont(f);
    l->setEnabled(false);
    l->setContentsMargins(0, 6, 0, 0);
    return l;
}

}  // namespace

PropertyFormWidget::PropertyFormWidget(const PropertyGroup& group, UnitSystem units, QWidget* parent)
    : QWidget(parent), groupId_(group.id), values_(group.values) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    form_ = new QFormLayout;
    form_->setContentsMargins(0, 0, 0, 0);
    form_->setHorizontalSpacing(10);
    form_->setVerticalSpacing(6);
    form_->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    form_->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    outer->addLayout(form_);

    // Group sub-headings in order of first appearance; a single group named like the form needs none.
    std::vector<std::string> groups;
    for (const auto& p : group.schema.properties)
        if (std::find(groups.begin(), groups.end(), p.group) == groups.end()) groups.push_back(p.group);
    const bool showHeadings = groups.size() > 1 || (groups.size() == 1 && !groups[0].empty() && qs(groups[0]) != group.title);

    std::vector<const sdk::PropertySpec*> ordered;
    for (const auto& g : groups)
        for (const auto& p : group.schema.properties)
            if (p.group == g && !p.advanced) ordered.push_back(&p);
    for (const auto& g : groups)
        for (const auto& p : group.schema.properties)
            if (p.group == g && p.advanced) ordered.push_back(&p);

    std::string currentGroup = "\x01";
    bool advancedHeadingShown = false;
    for (const sdk::PropertySpec* spec : ordered) {
        if (spec->advanced && !advancedHeadingShown) {
            advancedHeadingShown = true;
            advancedToggle_ = new QToolButton(this);
            advancedToggle_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
            advancedToggle_->setArrowType(Qt::RightArrow);
            advancedToggle_->setAutoRaise(true);
            advancedToggle_->setCheckable(true);
            form_->addRow(advancedToggle_);
            connect(advancedToggle_, &QToolButton::toggled, this, [this](bool on) {
                showAdvanced_ = on;
                advancedToggle_->setArrowType(on ? Qt::DownArrow : Qt::RightArrow);
                updateVisibility();
            });
            currentGroup = "\x01";
        } else if (showHeadings && !spec->advanced && spec->group != currentGroup && !spec->group.empty()) {
            form_->addRow(subheading(qs(spec->group), this));
        }
        currentGroup = spec->group;
        if (spec->advanced) ++advancedCount_;

        Row row;
        row.spec = *spec;
        row.editor = createFieldEditor(*spec, units, this);
        if (row.editor == nullptr) continue;
        auto* label = new QLabel(qs(spec->label), this);
        label->setToolTip(qs(spec->description));
        label->setWordWrap(true);
        row.label = label;
        row.container = new QWidget(this);
        auto* h = new QHBoxLayout(row.container);
        h->setContentsMargins(0, 0, 0, 0);
        h->setSpacing(4);
        h->addWidget(row.editor, 1);
        row.badge = new QLabel(row.container);
        row.badge->setVisible(false);
        h->addWidget(row.badge);
        row.stacked = spec->type == sdk::PropertyType::Vec3 || spec->type == sdk::PropertyType::Pose3 || spec->type == sdk::PropertyType::Box3;
        if (row.stacked) {
            form_->addRow(label);
            form_->addRow(row.container);
        } else {
            form_->addRow(label, row.container);
        }

        const QString key = qs(spec->key);
        connect(row.editor, &FieldEditor::edited, this, [this, key](const sdk::Value& v) {
            values_[key.toStdString()] = v;
            updateVisibility();
            Q_EMIT valueEdited(qs(groupId_), key, v);
        });
        rows_.push_back(std::move(row));
    }
    if (advancedToggle_ != nullptr) advancedToggle_->setText(ETR("Advanced (%1)").arg(advancedCount_));
    setValues(values_);
}

bool PropertyFormWidget::conditionMet(const sdk::PropertySpec& spec) const {
    if (spec.visibleWhenKey.empty()) return true;
    const sdk::Value* v = values_.find(spec.visibleWhenKey);
    return v != nullptr && *v == spec.visibleWhenEquals;
}

void PropertyFormWidget::updateVisibility() {
    for (const Row& r : rows_) {
        const bool visible = conditionMet(r.spec) && (!r.spec.advanced || showAdvanced_);
        if (r.stacked) {
            r.label->setVisible(visible);
            r.container->setVisible(visible);
        } else {
            form_->setRowVisible(r.container, visible);
        }
    }
}

void PropertyFormWidget::setValues(const sdk::Value& values) {
    values_ = values;
    for (Row& r : rows_) {
        QWidget* focused = QApplication::focusWidget();
        if (focused != nullptr && (focused == r.editor || r.editor->isAncestorOf(focused))) continue;   // never fight typing
        const sdk::Value* v = values.find(r.spec.key);
        r.editor->setValue(v != nullptr ? *v : r.spec.defaultValue);
    }
    updateVisibility();
}

void PropertyFormWidget::setFieldIssues(const QMap<QString, QPair<int, QString>>& issues) {
    for (Row& r : rows_) {
        const auto it = issues.find(qs(r.spec.key));
        if (it == issues.end()) {
            r.badge->setVisible(false);
            continue;
        }
        r.badge->setText(it->first == 2 ? QStringLiteral("\xE2\x9B\x94") : QStringLiteral("\xE2\x9A\xA0"));
        r.badge->setToolTip(it->second);
        r.badge->setVisible(true);
    }
}

bool PropertyFormWidget::hasField(const QString& key) const {
    return std::any_of(rows_.begin(), rows_.end(), [&](const Row& r) { return qs(r.spec.key) == key; });
}

FieldEditor* PropertyFormWidget::editorFor(const QString& key) const {
    for (const Row& r : rows_)
        if (qs(r.spec.key) == key) return r.editor;
    return nullptr;
}

int PropertyFormWidget::visibleRowCount() const {
    int n = 0;
    for (const Row& r : rows_)
        if (r.container->isVisibleTo(this) && (r.stacked || form_->isRowVisible(r.container))) ++n;
    return n;
}

bool PropertyFormWidget::focusField(const QString& key) {
    for (Row& r : rows_) {
        if (qs(r.spec.key) != key) continue;
        if (r.spec.advanced && advancedToggle_ != nullptr) advancedToggle_->setChecked(true);
        updateVisibility();
        r.editor->focusTarget()->setFocus(Qt::OtherFocusReason);
        for (QWidget* w = r.container->parentWidget(); w != nullptr; w = w->parentWidget())
            if (auto* area = qobject_cast<QScrollArea*>(w)) {
                area->ensureWidgetVisible(r.container, 0, 40);
                break;
            }
        QWidget* container = r.container;
        container->setStyleSheet(QStringLiteral("background: rgba(255, 190, 0, 70); border-radius: 4px;"));
        QTimer::singleShot(1400, container, [container] { container->setStyleSheet(QString()); });
        return true;
    }
    return false;
}

}  // namespace wpiemu::app::editor
