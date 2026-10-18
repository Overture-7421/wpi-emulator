#include "views/activation_card.h"

#include <QApplication>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <algorithm>

#include "model/config_ops.h"
#include "model/tr.h"

namespace wpiemu::app::editor {

namespace {

QString qs(const std::string& s) { return QString::fromStdString(s); }

bool hasFocusWithin(const QWidget* w) {
    const QWidget* focused = QApplication::focusWidget();
    return focused != nullptr && (focused == w || w->isAncestorOf(focused));
}

QString sevText(int severity) { return severity >= 2 ? QStringLiteral("\xE2\x9B\x94") : QStringLiteral("\xE2\x9A\xA0"); }

}  // namespace

ActivationCard::ActivationCard(EditorController& controller, const std::string& zoneId, QWidget* parent)
    : QWidget(parent), controller_(controller), zoneId_(zoneId) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);

    auto* head = new QWidget(this);
    auto* h = new QHBoxLayout(head);
    h->setContentsMargins(0, 0, 0, 0);
    h->addWidget(new QLabel(ETR("Active when"), head));
    mode_ = new QComboBox(head);
    mode_->setObjectName(QStringLiteral("activation.mode"));
    mode_->addItem(ETR("all conditions hold"), static_cast<int>(sdk::ActivationMode::All));
    mode_->addItem(ETR("any condition holds"), static_cast<int>(sdk::ActivationMode::Any));
    mode_->setToolTip(ETR("All: every condition must hold (AND). Any: one is enough (OR)."));
    h->addWidget(mode_, 1);
    layout->addWidget(head);
    connect(mode_, &QComboBox::currentIndexChanged, this, [this](int) {
        controller_.document().setZoneActivationMode(qs(zoneId_), static_cast<sdk::ActivationMode>(mode_->currentData().toInt()));
    });

    empty_ = new QLabel(ETR("Always active"), this);
    empty_->setObjectName(QStringLiteral("activation.empty"));
    empty_->setEnabled(false);
    layout->addWidget(empty_);

    rowsHost_ = new QWidget(this);
    auto* rowsLayout = new QVBoxLayout(rowsHost_);
    rowsLayout->setContentsMargins(0, 0, 0, 0);
    rowsLayout->setSpacing(6);
    rowsLayout_ = rowsLayout;
    layout->addWidget(rowsHost_);

    auto* add = new QToolButton(this);
    add->setObjectName(QStringLiteral("activation.add"));
    add->setText(ETR("+ Add condition"));
    add->setAutoRaise(true);
    add->setToolTip(ETR("Add a test on a value a mechanism publishes, e.g. a flywheel's velocity."));
    connect(add, &QToolButton::clicked, this, [this] {
        const int index = controller_.document().addZoneCondition(qs(zoneId_));
        if (index >= 0) focusCondition(index);
    });
    auto* addRow = new QHBoxLayout;
    addRow->addWidget(add);
    addRow->addStretch();
    layout->addLayout(addRow);

    refresh(true);
}

const sdk::ZoneConfig* ActivationCard::zone() const { return controller_.document().config().findZone(zoneId_); }

void ActivationCard::rebuildRows() {
    for (Row& r : rows_) {
        r.frame->hide();
        r.frame->setParent(nullptr);   // out of findChild() immediately; the card may be rebuilt from one of its own buttons
        r.frame->deleteLater();
    }
    rows_.clear();
    const sdk::ZoneConfig* z = zone();
    if (z == nullptr) return;
    for (int i = 0; i < static_cast<int>(z->activation.conditions.size()); ++i) {
        rows_.push_back(makeRow(i));
        rowsLayout_->addWidget(rows_.back().frame);
    }
}

ActivationCard::Row ActivationCard::makeRow(int index) {
    const QString n = QString::number(index);
    Row r;
    r.frame = new QFrame(rowsHost_);
    r.frame->setObjectName(QStringLiteral("activation.row:") + n);
    r.frame->setFrameShape(QFrame::StyledPanel);
    auto* g = new QGridLayout(r.frame);
    g->setContentsMargins(6, 6, 6, 6);
    g->setHorizontalSpacing(6);
    g->setVerticalSpacing(4);

    r.mechanism = new QComboBox(r.frame);
    r.mechanism->setObjectName(QStringLiteral("activation.mechanism:") + n);
    r.mechanism->setToolTip(ETR("Mechanism whose published value is tested."));
    g->addWidget(r.mechanism, 0, 0, 1, 3);

    r.remove = new QToolButton(r.frame);
    r.remove->setObjectName(QStringLiteral("activation.remove:") + n);
    r.remove->setText(QStringLiteral("\xE2\x9C\x95"));
    r.remove->setAutoRaise(true);
    r.remove->setToolTip(ETR("Remove this condition"));
    g->addWidget(r.remove, 0, 3);

    r.value = new QLineEdit(r.frame);
    r.value->setObjectName(QStringLiteral("activation.value:") + n);
    r.value->setPlaceholderText(ETR("value, e.g. velocity"));
    r.value->setToolTip(ETR("Name of the published value: velocity, position, ..."));
    g->addWidget(r.value, 1, 0, 1, 2);

    r.op = new QComboBox(r.frame);
    r.op->setObjectName(QStringLiteral("activation.op:") + n);
    r.op->addItem(ETR("Above"), static_cast<int>(sdk::ConditionOp::Above));
    r.op->addItem(ETR("Below"), static_cast<int>(sdk::ConditionOp::Below));
    r.op->addItem(ETR("Within"), static_cast<int>(sdk::ConditionOp::Within));
    g->addWidget(r.op, 1, 2, 1, 2);

    r.threshold = new QDoubleSpinBox(r.frame);
    r.threshold->setObjectName(QStringLiteral("activation.threshold:") + n);
    r.threshold->setRange(-1e6, 1e6);
    r.threshold->setDecimals(3);
    r.threshold->setToolTip(ETR("Threshold in SI units (m, rad, m/s, rad/s)."));
    g->addWidget(r.threshold, 2, 0, 1, 2);

    r.tolerance = new QDoubleSpinBox(r.frame);
    r.tolerance->setObjectName(QStringLiteral("activation.tolerance:") + n);
    r.tolerance->setRange(0.0, 1e6);
    r.tolerance->setDecimals(3);
    r.tolerance->setPrefix(QStringLiteral("\xC2\xB1 "));
    r.tolerance->setToolTip(ETR("Within: the value may differ from the threshold by this much."));
    g->addWidget(r.tolerance, 2, 2, 1, 2);

    r.absolute = new QCheckBox(ETR("absolute value"), r.frame);
    r.absolute->setObjectName(QStringLiteral("activation.absolute:") + n);
    r.absolute->setToolTip(ETR("Compare the magnitude, so a roller spinning backwards counts too."));
    g->addWidget(r.absolute, 3, 0, 1, 3);

    r.badge = new QLabel(r.frame);
    r.badge->setObjectName(QStringLiteral("activation.badge:") + n);
    r.badge->setVisible(false);
    g->addWidget(r.badge, 3, 3);

    connect(r.mechanism, &QComboBox::currentIndexChanged, this, [this, index](int) { commit(index); });
    connect(r.value, &QLineEdit::editingFinished, this, [this, index] { commit(index); });
    connect(r.op, &QComboBox::currentIndexChanged, this, [this, index](int) { commit(index); });
    connect(r.threshold, &QDoubleSpinBox::valueChanged, this, [this, index](double) { commit(index); });
    connect(r.tolerance, &QDoubleSpinBox::valueChanged, this, [this, index](double) { commit(index); });
    connect(r.absolute, &QCheckBox::toggled, this, [this, index](bool) { commit(index); });
    connect(r.remove, &QToolButton::clicked, this, [this, index] { controller_.document().removeZoneCondition(qs(zoneId_), index); });
    return r;
}

sdk::ZoneCondition ActivationCard::conditionOf(const Row& r) const {
    sdk::ZoneCondition c;
    c.mechanismId = r.mechanism->currentData().toString().toStdString();
    c.value = r.value->text().trimmed().toStdString();
    c.op = static_cast<sdk::ConditionOp>(r.op->currentData().toInt());
    c.threshold = r.threshold->value();
    c.tolerance = r.tolerance->value();
    c.absolute = r.absolute->isChecked();
    return c;
}

void ActivationCard::commit(int index) {
    if (index < 0 || index >= static_cast<int>(rows_.size())) return;
    Row& r = rows_[static_cast<std::size_t>(index)];
    const sdk::ZoneCondition condition = conditionOf(r);
    const bool within = condition.op == sdk::ConditionOp::Within;
    r.tolerance->setVisible(within);
    r.absolute->setVisible(!within);
    controller_.document().setZoneCondition(qs(zoneId_), index, condition);
}

void ActivationCard::syncMechanisms(Row& r, const std::string& selected) {
    const sdk::RobotConfig& c = controller_.document().config();
    const EditorCatalog& cat = controller_.document().catalog();
    std::vector<std::pair<QString, QString>> items;   // (label, id)
    for (const auto& m : c.mechanisms) items.emplace_back(qs(ops::displayNameFor(m, cat)), qs(m.id));
    const bool known = std::any_of(c.mechanisms.begin(), c.mechanisms.end(), [&](const sdk::MechanismConfig& m) { return m.id == selected; });
    if (!known) items.emplace_back(selected.empty() ? ETR("(choose a mechanism)") : qs(selected) + QStringLiteral(" ") + ETR("(missing)"), qs(selected));

    bool same = r.mechanism->count() == static_cast<int>(items.size());
    for (int i = 0; same && i < r.mechanism->count(); ++i)
        same = r.mechanism->itemData(i).toString() == items[static_cast<std::size_t>(i)].second &&
               r.mechanism->itemText(i) == items[static_cast<std::size_t>(i)].first;
    if (!same) {
        r.mechanism->clear();
        for (const auto& [label, id] : items) r.mechanism->addItem(label, id);
    }
    r.mechanism->setCurrentIndex(std::max(0, r.mechanism->findData(qs(selected))));
}

void ActivationCard::syncRow(Row& r, const sdk::ZoneCondition& c, bool authoritative) {
    auto skip = [&](QWidget* w) { return !authoritative && hasFocusWithin(w); };
    if (!skip(r.mechanism)) {
        const QSignalBlocker b(r.mechanism);
        syncMechanisms(r, c.mechanismId);
    }
    if (!skip(r.value)) {
        const QSignalBlocker b(r.value);
        if (r.value->text().trimmed() != qs(c.value)) r.value->setText(qs(c.value));
    }
    if (!skip(r.op)) {
        const QSignalBlocker b(r.op);
        r.op->setCurrentIndex(r.op->findData(static_cast<int>(c.op)));
    }
    if (!skip(r.threshold)) {
        const QSignalBlocker b(r.threshold);
        r.threshold->setValue(c.threshold);
    }
    if (!skip(r.tolerance)) {
        const QSignalBlocker b(r.tolerance);
        r.tolerance->setValue(c.tolerance);
    }
    {
        const QSignalBlocker b(r.absolute);
        r.absolute->setChecked(c.absolute);
    }
    const bool within = c.op == sdk::ConditionOp::Within;
    r.tolerance->setVisible(within);
    r.absolute->setVisible(!within);
}

void ActivationCard::refresh(bool authoritative) {
    const sdk::ZoneConfig* z = zone();
    if (z == nullptr) return;
    {
        const QSignalBlocker b(mode_);
        mode_->setCurrentIndex(mode_->findData(static_cast<int>(z->activation.mode)));
    }
    if (static_cast<int>(rows_.size()) != static_cast<int>(z->activation.conditions.size())) rebuildRows();
    for (std::size_t i = 0; i < rows_.size(); ++i) syncRow(rows_[i], z->activation.conditions[i], authoritative);
    empty_->setVisible(rows_.empty());
    setIssues(issues_);
}

void ActivationCard::setIssues(const QMap<QString, QPair<int, QString>>& issues) {
    issues_ = issues;
    for (std::size_t i = 0; i < rows_.size(); ++i) {
        const auto it = issues.find(QStringLiteral("activation:%1").arg(i));
        rows_[i].badge->setVisible(it != issues.end());
        if (it != issues.end()) {
            rows_[i].badge->setText(sevText(it->first));
            rows_[i].badge->setToolTip(it->second);
        }
    }
}

bool ActivationCard::focusCondition(int index) {
    if (index < 0 || index >= static_cast<int>(rows_.size())) return false;
    Row& r = rows_[static_cast<std::size_t>(index)];
    r.value->setFocus(Qt::OtherFocusReason);
    for (QWidget* w = r.frame->parentWidget(); w != nullptr; w = w->parentWidget())
        if (auto* area = qobject_cast<QScrollArea*>(w)) {
            area->ensureWidgetVisible(r.frame, 0, 40);
            break;
        }
    return true;
}

}  // namespace wpiemu::app::editor
