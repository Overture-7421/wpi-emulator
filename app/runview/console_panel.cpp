#include "console_panel.h"

#include <QAction>
#include <QApplication>
#include <QClipboard>
#include <QComboBox>
#include <QFontDatabase>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListView>
#include <QMenu>
#include <QScrollBar>
#include <QShortcut>
#include <QStyledItemDelegate>
#include <QToolButton>
#include <QVBoxLayout>

#include "status_chip.h"

namespace wpiemu::app::runview {
namespace {

class LevelDelegate : public QStyledItemDelegate {
public:
    using QStyledItemDelegate::QStyledItemDelegate;

protected:
    void initStyleOption(QStyleOptionViewItem* option, const QModelIndex& index) const override {
        QStyledItemDelegate::initStyleOption(option, index);
        if (option->state & QStyle::State_Selected) return;
        switch (static_cast<sdk::LogLevel>(index.data(ConsoleModel::LevelRole).toInt())) {
            case sdk::LogLevel::Error: option->palette.setColor(QPalette::Text, toneColor(Tone::Error)); break;
            case sdk::LogLevel::Warn: option->palette.setColor(QPalette::Text, toneColor(Tone::Warning)); break;
            case sdk::LogLevel::Debug:
            case sdk::LogLevel::Trace:
                option->palette.setColor(QPalette::Text, option->palette.color(QPalette::Disabled, QPalette::Text));
                break;
            case sdk::LogLevel::Info: break;
        }
    }
};

}  // namespace

ConsolePanel::ConsolePanel(QWidget* parent) : QWidget(parent), model_(nullptr), filter_(nullptr) {
    setObjectName(QStringLiteral("runview.console"));
    filter_.setSourceModel(&model_);

    auto* title = new QLabel(tr("Console"), this);
    QFont bold = title->font();
    bold.setBold(true);
    title->setFont(bold);

    levelCombo_ = new QComboBox(this);
    levelCombo_->setToolTip(tr("Show messages of at least this level"));
    levelCombo_->addItem(tr("All levels"), static_cast<int>(sdk::LogLevel::Trace));
    levelCombo_->addItem(tr("Debug and up"), static_cast<int>(sdk::LogLevel::Debug));
    levelCombo_->addItem(tr("Info and up"), static_cast<int>(sdk::LogLevel::Info));
    levelCombo_->addItem(tr("Warnings and errors"), static_cast<int>(sdk::LogLevel::Warn));
    levelCombo_->addItem(tr("Errors only"), static_cast<int>(sdk::LogLevel::Error));

    streams_ = new QToolButton(this);
    streams_->setText(tr("Sources"));
    streams_->setToolTip(tr("Choose which output streams are shown"));
    streams_->setPopupMode(QToolButton::InstantPopup);
    streams_->setStyleSheet(QStringLiteral("QToolButton { padding-right: 16px; }"));
    auto* menu = new QMenu(streams_);
    const std::pair<sdk::LogStream, QString> streamNames[] = {
        {sdk::LogStream::Stdout, tr("Robot output (stdout)")},
        {sdk::LogStream::Stderr, tr("Robot errors (stderr)")},
        {sdk::LogStream::Emulator, tr("Emulator messages")}};
    for (const auto& [stream, text] : streamNames) {
        QAction* a = menu->addAction(text);
        a->setCheckable(true);
        a->setChecked(true);
        connect(a, &QAction::toggled, this, [this, stream = stream](bool on) { filter_.setStreamEnabled(stream, on); updateCount(); });
    }
    streams_->setMenu(menu);

    search_ = new QLineEdit(this);
    search_->setPlaceholderText(tr("Search console…"));
    search_->setClearButtonEnabled(true);
    search_->setAccessibleName(tr("Search console"));

    count_ = new QLabel(this);
    count_->setEnabled(false);

    auto* copyButton = new QToolButton(this);
    copyButton->setText(tr("Copy"));
    copyButton->setToolTip(tr("Copy the selected lines, or all visible lines (Ctrl+C)"));
    auto* clearButton = new QToolButton(this);
    clearButton->setText(tr("Clear"));
    clearButton->setToolTip(tr("Remove all lines from the console"));
    follow_button_ = new QToolButton(this);
    follow_button_->setText(tr("Follow"));
    follow_button_->setCheckable(true);
    follow_button_->setChecked(true);
    follow_button_->setToolTip(tr("Keep the newest line in view"));

    auto* bar = new QHBoxLayout;
    bar->setContentsMargins(0, 0, 0, 0);
    bar->addWidget(title);
    bar->addSpacing(8);
    bar->addWidget(levelCombo_);
    bar->addWidget(streams_);
    bar->addWidget(search_, 1);
    bar->addWidget(count_);
    bar->addWidget(follow_button_);
    bar->addWidget(copyButton);
    bar->addWidget(clearButton);

    view_ = new QListView(this);
    view_->setModel(&filter_);
    view_->setItemDelegate(new LevelDelegate(view_));
    view_->setUniformItemSizes(true);
    view_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    view_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    view_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    view_->setWordWrap(false);
    view_->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    view_->setAlternatingRowColors(false);
    view_->setAccessibleName(tr("Robot console output"));

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 6, 8, 8);
    layout->setSpacing(6);
    layout->addLayout(bar);
    layout->addWidget(view_, 1);

    connect(levelCombo_, &QComboBox::currentIndexChanged, this, [this](int i) {
        filter_.setMinimumLevel(static_cast<sdk::LogLevel>(levelCombo_->itemData(i).toInt()));
        updateCount();
    });
    connect(search_, &QLineEdit::textChanged, this, [this](const QString& t) { filter_.setSearchText(t); updateCount(); });
    connect(copyButton, &QToolButton::clicked, this, &ConsolePanel::copy);
    connect(clearButton, &QToolButton::clicked, this, &ConsolePanel::clear);
    connect(follow_button_, &QToolButton::toggled, this, &ConsolePanel::setFollowTail);
    connect(&filter_, &QAbstractItemModel::rowsInserted, this, [this] { updateCount(); scrollToTailIfFollowing(); });
    connect(&filter_, &QAbstractItemModel::rowsRemoved, this, [this] { updateCount(); });
    connect(&filter_, &QAbstractItemModel::modelReset, this, [this] { updateCount(); });
    connect(&filter_, &QAbstractItemModel::layoutChanged, this, [this] { updateCount(); });

    auto* copyShortcut = new QShortcut(QKeySequence::Copy, view_);
    copyShortcut->setContext(Qt::WidgetShortcut);
    connect(copyShortcut, &QShortcut::activated, this, &ConsolePanel::copy);
    auto* findShortcut = new QShortcut(QKeySequence::Find, this);
    findShortcut->setContext(Qt::WidgetWithChildrenShortcut);
    connect(findShortcut, &QShortcut::activated, search_, qOverload<>(&QLineEdit::setFocus));

    updateCount();
}

void ConsolePanel::append(const HostLogLine& entry) { model_.append(entry); }

void ConsolePanel::clear() { model_.clear(); }

void ConsolePanel::setFollowTail(bool follow) {
    follow_ = follow;
    if (follow_button_ != nullptr) follow_button_->setChecked(follow);
    if (follow_) scrollToTailIfFollowing();
}

void ConsolePanel::copy() {
    QApplication::clipboard()->setText(filter_.plainText(view_->selectionModel()->selectedRows()));
}

void ConsolePanel::updateCount() {
    count_->setText(filter_.rowCount() == model_.rowCount() ? tr("%n line(s)", nullptr, model_.rowCount())
                                                              : tr("%1 of %2 lines").arg(filter_.rowCount()).arg(model_.rowCount()));
}

void ConsolePanel::scrollToTailIfFollowing() {
    if (!follow_ || scrollQueued_) return;
    scrollQueued_ = true;
    QMetaObject::invokeMethod(this, [this] {
        scrollQueued_ = false;
        if (follow_) view_->scrollToBottom();
    }, Qt::QueuedConnection);
}

}  // namespace wpiemu::app::runview
