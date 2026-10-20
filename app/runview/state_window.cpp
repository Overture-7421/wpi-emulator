#include "state_window.h"

#include <QCloseEvent>
#include <QFontDatabase>
#include <QHeaderView>
#include <QLabel>
#include <QTreeView>
#include <QVBoxLayout>

namespace wpiemu::app::runview {

StateWindow::StateWindow(const ViewportTheme& theme, QWidget* parent) : QWidget(parent, Qt::Window) {
    setObjectName(QStringLiteral("runview.stateWindow"));
    setWindowTitle(tr("Robot state"));
    resize(460, 560);

    model_ = new StateModel(theme, this);

    tree_ = new QTreeView(this);
    tree_->setObjectName(QStringLiteral("runview.stateTree"));
    tree_->setModel(model_);
    tree_->setAlternatingRowColors(true);
    tree_->setUniformRowHeights(true);
    tree_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    tree_->setSelectionMode(QAbstractItemView::SingleSelection);
    tree_->setIndentation(16);
    tree_->header()->setStretchLastSection(false);
    tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    tree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);

    empty_ = new QLabel(this);
    empty_->setObjectName(QStringLiteral("runview.stateEmpty"));
    empty_->setAlignment(Qt::AlignCenter);
    empty_->setWordWrap(true);
    empty_->setEnabled(false);

    footer_ = new QLabel(this);
    footer_->setObjectName(QStringLiteral("runview.stateFooter"));
    footer_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    footer_->setEnabled(false);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(6);
    layout->addWidget(tree_, 1);
    layout->addWidget(empty_, 1);
    layout->addWidget(footer_);

    connect(model_, &QAbstractItemModel::modelReset, tree_, &QTreeView::expandAll);
    refreshEmptyState(false);
}

void StateWindow::showSnapshot(const sdk::ISimSnapshot* snapshot) {
    model_->update(snapshot);
    refreshEmptyState(snapshot != nullptr);
    footer_->setText(snapshot != nullptr ? tr("Sim time %1 s").arg(snapshot->simTimeUs() / 1e6, 0, 'f', 2) : QString());
}

void StateWindow::refreshEmptyState(bool haveSnapshot) {
    const bool empty = model_->isEmpty();
    tree_->setVisible(!empty);
    empty_->setVisible(empty);
    empty_->setText(haveSnapshot ? tr("The robot has no mechanisms, sensors, zones or cameras to show.")
                                 : tr("Start the simulation to see live state."));
}

void StateWindow::closeEvent(QCloseEvent* e) {
    QWidget::closeEvent(e);
    if (e->isAccepted()) emit closed();
}

}  // namespace wpiemu::app::runview
