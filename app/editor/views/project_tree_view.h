#pragma once

// Tree view for the project model: custom row painting (icons, status dots, muted detail text),
// drag-and-drop feedback (green = the row accepts the dragged device / template, red = it does not),
// and expansion state that survives model rebuilds.

#include <QSet>
#include <QStyledItemDelegate>
#include <QTreeView>

#include "viewmodel/project_tree_model.h"
#include "views/icon_factory.h"

namespace wpiemu::app::editor {

class ProjectTreeDelegate : public QStyledItemDelegate {
    Q_OBJECT
public:
    ProjectTreeDelegate(const IconFactory& icons, QObject* parent = nullptr) : QStyledItemDelegate(parent), icons_(icons) {}
    void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    QSize sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const override;

private:
    const IconFactory& icons_;
};

class ProjectTreeView : public QTreeView {
    Q_OBJECT
public:
    ProjectTreeView(ProjectTreeModel& model, const IconFactory& icons, QWidget* parent = nullptr);

    void selectRef(const SelectionRef& ref);

Q_SIGNALS:
    void refSelected(const wpiemu::app::editor::SelectionRef& ref);
    void contextMenuRequestedFor(const wpiemu::app::editor::SelectionRef& ref, const QPoint& globalPos);

protected:
    void dragEnterEvent(QDragEnterEvent* event) override;
    void dragMoveEvent(QDragMoveEvent* event) override;
    void dragLeaveEvent(QDragLeaveEvent* event) override;
    void dropEvent(QDropEvent* event) override;
    void paintEvent(QPaintEvent* event) override;
    void contextMenuEvent(QContextMenuEvent* event) override;

private:
    void saveExpansion();
    void restoreExpansion();
    void clearDropHint();

    ProjectTreeModel& model_;
    QSet<QString> expanded_;
    bool firstReset_ = true;
    bool syncing_ = false;
    QModelIndex dropIndex_;
    bool dropOk_ = false;
};

}  // namespace wpiemu::app::editor
