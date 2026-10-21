#include "views/bottom_panel.h"

#include <QDateTime>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QStackedLayout>
#include <QStyledItemDelegate>
#include <QVBoxLayout>

#include "model/tr.h"
#include "ui/qt_text.h"
#include "ui/widget_support.h"

namespace wpiemu::app::editor {

namespace {

// Oldest console lines are dropped beyond this many.
constexpr int kMaxConsoleLines = 5000;

class IssueDelegate : public QStyledItemDelegate {
public:
    IssueDelegate(const IconFactory& icons, QObject* parent) : QStyledItemDelegate(parent), icons_(icons) {}
    void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override {
        QStyleOptionViewItem opt = option;
        initStyleOption(&opt, index);
        if (index.column() == IssueModel::ColSeverity) {
            opt.icon = icons_.severity(index.data(IssueModel::SeverityRole).toInt());
            opt.decorationSize = QSize(16, 16);
            opt.features |= QStyleOptionViewItem::HasDecoration;
        }
        QStyledItemDelegate::paint(painter, opt, index);
    }

private:
    const IconFactory& icons_;
};

QWidget* emptyState(const QString& text, QWidget* parent) {
    QLabel* l = ui::mutedLabel(text, parent);
    l->setAlignment(Qt::AlignCenter);
    return l;
}

// Table + an empty-state label, swapped by the caller.
QWidget* stacked(QWidget* content, QWidget* empty, QWidget* parent) {
    auto* host = new QWidget(parent);
    auto* s = new QStackedLayout(host);
    s->addWidget(empty);
    s->addWidget(content);
    return host;
}

}  // namespace

BottomPanel::BottomPanel(EditorController& controller, IssueModel& issues, const IconFactory& icons, QWidget* parent)
    : QTabWidget(parent), controller_(controller), issues_(issues), icons_(icons) {
    setDocumentMode(true);
    setTabPosition(QTabWidget::North);

    // Console
    console_ = new QPlainTextEdit(this);
    console_->setReadOnly(true);
    console_->setMaximumBlockCount(kMaxConsoleLines);
    console_->setLineWrapMode(QPlainTextEdit::NoWrap);
    QFont mono = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    console_->setFont(mono);
    console_->setPlaceholderText(ETR("Robot and discovery output appears here."));
    console_->setFrameShape(QFrame::NoFrame);
    addTab(console_, ETR("Console"));

    // Validation
    issueTable_ = new QTableView(this);
    issueTable_->setModel(&issues_);
    issueTable_->setItemDelegate(new IssueDelegate(icons_, issueTable_));
    issueTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    issueTable_->setSelectionMode(QAbstractItemView::SingleSelection);
    issueTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    issueTable_->setShowGrid(false);
    issueTable_->setAlternatingRowColors(true);
    issueTable_->verticalHeader()->hide();
    issueTable_->verticalHeader()->setDefaultSectionSize(24);
    issueTable_->horizontalHeader()->setStretchLastSection(true);
    issueTable_->horizontalHeader()->setSectionResizeMode(IssueModel::ColSeverity, QHeaderView::Fixed);
    issueTable_->setColumnWidth(IssueModel::ColSeverity, 78);
    issueTable_->horizontalHeader()->setSectionResizeMode(IssueModel::ColMessage, QHeaderView::Stretch);
    issueTable_->setColumnWidth(IssueModel::ColWhere, 220);
    issueTable_->setFrameShape(QFrame::NoFrame);
    issueEmpty_ = emptyState(ETR("No problems found. Required slots are mapped and the model validates."), this);
    addTab(stacked(issueTable_, issueEmpty_, this), ETR("Validation"));
    validationTabIndex_ = 1;

    connect(issueTable_, &QTableView::clicked, this, [this](const QModelIndex& index) {
        if (const sdk::Issue* issue = issues_.issueAt(index)) controller_.focusIssue(*issue);
    });
    connect(issueTable_, &QTableView::activated, this, [this](const QModelIndex& index) {
        if (const sdk::Issue* issue = issues_.issueAt(index)) controller_.focusIssue(*issue);
    });

    // Telemetry
    telemetry_ = new QTableWidget(0, 2, this);
    telemetry_->setHorizontalHeaderLabels({ETR("Signal"), ETR("Value")});
    telemetry_->horizontalHeader()->setStretchLastSection(true);
    telemetry_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    telemetry_->verticalHeader()->hide();
    telemetry_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    telemetry_->setShowGrid(false);
    telemetry_->setAlternatingRowColors(true);
    telemetry_->setFrameShape(QFrame::NoFrame);
    telemetryEmpty_ = emptyState(ETR("Live telemetry shows up here while a robot is connected (use \"Detect devices\" and run your robot code)."), this);
    addTab(stacked(telemetry_, telemetryEmpty_, this), ETR("Telemetry"));

    auto* clear = new QPushButton(ETR("Clear"), this);
    clear->setFlat(true);
    connect(clear, &QPushButton::clicked, console_, &QPlainTextEdit::clear);
    setCornerWidget(clear, Qt::TopRightCorner);

    connect(&issues_, &QAbstractItemModel::modelReset, this, [this] { updateValidationTab(); });
    updateValidationTab();
}

void BottomPanel::showValidation() { setCurrentIndex(validationTabIndex_); }

void BottomPanel::updateValidationTab() {
    const int e = issues_.count(sdk::Severity::Error);
    const int w = issues_.count(sdk::Severity::Warning);
    const int total = issues_.rowCount();
    QString title = ETR("Validation");
    if (total > 0) title += QStringLiteral(" (%1)").arg(total);
    setTabText(validationTabIndex_, title);
    setTabIcon(validationTabIndex_, total == 0 ? QIcon() : icons_.severity(e > 0 ? 2 : w > 0 ? 1 : 0));
    setTabToolTip(validationTabIndex_, issues_.summary());
    auto* host = widget(validationTabIndex_);
    if (auto* layout = qobject_cast<QStackedLayout*>(host->layout())) layout->setCurrentIndex(total == 0 ? 0 : 1);
}

void BottomPanel::appendLog(sdk::Severity severity, const QString& text) {
    const QString color = severity == sdk::Severity::Error ? QStringLiteral("#f85149") : severity == sdk::Severity::Warning ? QStringLiteral("#d29922") : QString();
    const QString time = ui::clockText(QDateTime::currentMSecsSinceEpoch());
    const QString line = QStringLiteral("<span style=\"color:gray\">%1</span> %2").arg(time, text.toHtmlEscaped());
    console_->appendHtml(color.isEmpty() ? line : QStringLiteral("<span style=\"color:%1\">%2</span>").arg(color, line));
}

void BottomPanel::setSnapshot(const std::shared_ptr<const sdk::ISimSnapshot>& snapshot) {
    auto* layout = qobject_cast<QStackedLayout*>(widget(2)->layout());
    if (!snapshot) {
        if (layout != nullptr) layout->setCurrentIndex(0);
        return;
    }
    if (layout != nullptr) layout->setCurrentIndex(1);
    std::vector<std::pair<QString, QString>> rows;
    rows.emplace_back(ETR("Robot connected"), snapshot->robotConnected() ? ETR("yes") : ETR("no"));
    rows.emplace_back(ETR("Tick"), QString::number(snapshot->tick()));
    rows.emplace_back(ETR("Battery"), QStringLiteral("%1 V").arg(snapshot->batteryVoltageV(), 0, 'f', 2));
    rows.emplace_back(ETR("Devices"), QString::number(snapshot->devices().size()));
    for (const auto& m : snapshot->mechanisms())
        for (const auto& v : m.values)
            rows.emplace_back(QString::fromStdString(m.id + "." + v.name), QStringLiteral("%1 %2").arg(v.value, 0, 'g', 5).arg(QString::fromStdString(v.unit)));
    for (const auto& w : snapshot->warnings()) rows.emplace_back(ETR("Warning"), QString::fromStdString(w.message));
    telemetry_->setRowCount(static_cast<int>(rows.size()));
    for (std::size_t i = 0; i < rows.size(); ++i) {
        telemetry_->setItem(static_cast<int>(i), 0, new QTableWidgetItem(rows[i].first));
        telemetry_->setItem(static_cast<int>(i), 1, new QTableWidgetItem(rows[i].second));
    }
}

}  // namespace wpiemu::app::editor
