#include "views/project_tree_view.h"

#include <QApplication>
#include <QContextMenuEvent>
#include <QDragMoveEvent>
#include <QHeaderView>
#include <QPainter>
#include <QStyleOptionViewItem>

#include <functional>

namespace wpiemu::app::editor {

// ---- delegate ----------------------------------------------------------------------------------
QSize ProjectTreeDelegate::sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const {
    QSize s = QStyledItemDelegate::sizeHint(option, index);
    s.setHeight(std::max(s.height(), 24));
    return s;
}

void ProjectTreeDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const {
    painter->save();
    QStyleOptionViewItem opt = option;
    initStyleOption(&opt, index);
    const QString text = opt.text;
    opt.text.clear();
    opt.icon = QIcon();
    const QWidget* widget = opt.widget;
    QStyle* style = widget != nullptr ? widget->style() : QApplication::style();
    style->drawControl(QStyle::CE_ItemViewItem, &opt, painter, widget);

    const auto kind = static_cast<NodeKind>(index.data(ProjectTreeModel::KindRole).toInt());
    const int state = index.data(ProjectTreeModel::StateRole).toInt();
    const bool selected = opt.state & QStyle::State_Selected;
    const QString detail = index.data(ProjectTreeModel::DetailRole).toString();

    QRect r = opt.rect.adjusted(4, 0, -6, 0);
    QIcon icon = kind == NodeKind::Slot && state >= 0 ? icons_.status(static_cast<SlotState>(state)) : icons_.node(kind);
    if (!icon.isNull()) {
        icon.paint(painter, QRect(r.left(), r.center().y() - 8, 16, 16));
        r.setLeft(r.left() + 22);
    }

    QFont font = opt.font;
    const bool group = kind == NodeKind::Group;
    if (group || kind == NodeKind::Robot) font.setBold(true);
    const bool follower = index.data(ProjectTreeModel::FollowerRole).toBool();
    if (follower && !selected) painter->setOpacity(0.6);   // read-only: driven by a motion link
    painter->setFont(font);
    const QColor fg = selected ? opt.palette.color(QPalette::HighlightedText) : opt.palette.color(QPalette::Text);
    QColor muted = selected ? opt.palette.color(QPalette::HighlightedText) : opt.palette.color(QPalette::PlaceholderText);
    if (!selected && (kind == NodeKind::Slot || kind == NodeKind::Wheel) && state >= 0) {
        const auto st = static_cast<SlotState>(state);
        if (st == SlotState::Unmapped || st == SlotState::NotDetected || st == SlotState::Incompatible || st == SlotState::AliasConflict)
            muted = IconFactory::stateColor(st);
    }

    int detailWidth = 0;
    const bool warning = index.data(ProjectTreeModel::WarningRole).toBool();
    if (warning) {
        const int d = 8;
        painter->setRenderHint(QPainter::Antialiasing);
        painter->setBrush(QColor("#d29922"));
        painter->setPen(Qt::NoPen);
        painter->drawEllipse(QRect(r.right() - d, r.center().y() - d / 2, d, d));
        r.setRight(r.right() - d - 6);
    }
    if (!detail.isEmpty()) {
        QFont df = font;
        df.setBold(false);
        // The theme stylesheet sizes fonts in pixels, for which pointSizeF() is -1.
        if (df.pointSizeF() > 0) df.setPointSizeF(df.pointSizeF() * 0.92);
        else if (df.pixelSize() > 0) df.setPixelSize(std::max(1, qRound(df.pixelSize() * 0.92)));
        const QFontMetrics fm(df);
        const QFontMetrics tm(font);
        const int maxDetail = std::max(40, r.width() / 2);
        const QString elided = fm.elidedText(detail, Qt::ElideRight, maxDetail);
        detailWidth = fm.horizontalAdvance(elided);
        const int textRoom = r.width() - detailWidth - 10;
        painter->setFont(font);
        painter->setPen(fg);
        painter->drawText(QRect(r.left(), r.top(), std::max(textRoom, 20), r.height()), Qt::AlignVCenter | Qt::AlignLeft,
                          tm.elidedText(text, Qt::ElideRight, std::max(textRoom, 20)));
        painter->setFont(df);
        painter->setPen(muted);
        painter->drawText(QRect(r.right() - detailWidth, r.top(), detailWidth, r.height()), Qt::AlignVCenter | Qt::AlignRight, elided);
    } else {
        painter->setPen(fg);
        painter->drawText(r, Qt::AlignVCenter | Qt::AlignLeft, QFontMetrics(font).elidedText(text, Qt::ElideRight, r.width()));
    }
    painter->restore();
}

// ---- view ---------------------------------------------------------------------------------------
ProjectTreeView::ProjectTreeView(ProjectTreeModel& model, const IconFactory& icons, QWidget* parent) : QTreeView(parent), model_(model) {
    setModel(&model_);
    setItemDelegate(new ProjectTreeDelegate(icons, this));
    setHeaderHidden(true);
    setUniformRowHeights(true);
    setIndentation(14);
    setAnimated(false);
    setEditTriggers(QAbstractItemView::NoEditTriggers);
    setSelectionMode(QAbstractItemView::SingleSelection);
    setDragEnabled(true);
    setAcceptDrops(true);
    setDropIndicatorShown(false);
    setDragDropMode(QAbstractItemView::DragDrop);
    setDragDropOverwriteMode(true);
    setDefaultDropAction(Qt::CopyAction);
    setContextMenuPolicy(Qt::DefaultContextMenu);
    setAllColumnsShowFocus(true);
    setExpandsOnDoubleClick(true);

    connect(&model_, &QAbstractItemModel::modelAboutToBeReset, this, [this] { saveExpansion(); });
    connect(&model_, &QAbstractItemModel::modelReset, this, [this] { restoreExpansion(); });
    connect(this, &QTreeView::expanded, this, [this](const QModelIndex& i) { expanded_.insert(model_.keyAt(i)); });
    connect(this, &QTreeView::collapsed, this, [this](const QModelIndex& i) { expanded_.remove(model_.keyAt(i)); });
    connect(selectionModel(), &QItemSelectionModel::currentChanged, this, [this](const QModelIndex& current) {
        if (syncing_) return;
        const SelectionRef ref = model_.refAt(current);
        if (!ref.isNone()) Q_EMIT refSelected(ref);
    });
    restoreExpansion();
}

void ProjectTreeView::saveExpansion() {
    expanded_.clear();
    std::function<void(const QModelIndex&)> walk = [&](const QModelIndex& parent) {
        for (int r = 0; r < model_.rowCount(parent); ++r) {
            const QModelIndex i = model_.index(r, 0, parent);
            if (isExpanded(i)) expanded_.insert(model_.keyAt(i));
            walk(i);
        }
    };
    walk({});
}

void ProjectTreeView::restoreExpansion() {
    if (firstReset_ && model_.rowCount() > 0) {
        firstReset_ = false;
        expandToDepth(1);
        // Wheels, slots and the chassis' shapes are noisy until asked for.
        for (const QString& key : {QStringLiteral("drivetrain"), QStringLiteral("chassis")}) {
            const QModelIndex idx = model_.indexForKey(key);
            if (idx.isValid()) collapse(idx);
        }
        for (int r = 0; r < model_.rowCount(model_.index(0, 0)); ++r) {
            const QModelIndex i = model_.index(r, 0, model_.index(0, 0));
            if (isExpanded(i)) expanded_.insert(model_.keyAt(i));
        }
        return;
    }
    std::function<void(const QModelIndex&)> walk = [&](const QModelIndex& parent) {
        for (int r = 0; r < model_.rowCount(parent); ++r) {
            const QModelIndex i = model_.index(r, 0, parent);
            if (expanded_.contains(model_.keyAt(i))) setExpanded(i, true);
            walk(i);
        }
    };
    walk({});
}

void ProjectTreeView::selectRef(const SelectionRef& ref) {
    const QModelIndex idx = model_.indexFor(ref);
    syncing_ = true;
    if (idx.isValid()) {
        // Reveal the row: expand its ancestors.
        for (QModelIndex p = idx.parent(); p.isValid(); p = p.parent()) setExpanded(p, true);
        setCurrentIndex(idx);
        scrollTo(idx);
    } else {
        clearSelection();
        setCurrentIndex({});
    }
    syncing_ = false;
}

void ProjectTreeView::clearDropHint() {
    dropIndex_ = {};
    viewport()->update();
}

void ProjectTreeView::dragEnterEvent(QDragEnterEvent* event) {
    QTreeView::dragEnterEvent(event);
    event->acceptProposedAction();
}

void ProjectTreeView::dragMoveEvent(QDragMoveEvent* event) {
    const QModelIndex idx = indexAt(event->position().toPoint());
    dropIndex_ = idx;
    dropOk_ = model_.canDropMimeData(event->mimeData(), event->proposedAction(), -1, 0, idx);
    if (dropOk_) event->acceptProposedAction();
    else event->ignore();
    viewport()->update();
}

void ProjectTreeView::dragLeaveEvent(QDragLeaveEvent* event) {
    QTreeView::dragLeaveEvent(event);
    clearDropHint();
}

void ProjectTreeView::dropEvent(QDropEvent* event) {
    const QModelIndex idx = indexAt(event->position().toPoint());
    const bool ok = model_.dropMimeData(event->mimeData(), Qt::CopyAction, -1, 0, idx);
    clearDropHint();
    if (ok) event->acceptProposedAction();
    else event->ignore();
}

void ProjectTreeView::paintEvent(QPaintEvent* event) {
    QTreeView::paintEvent(event);
    if (!dropIndex_.isValid()) return;
    QPainter p(viewport());
    p.setRenderHint(QPainter::Antialiasing);
    const QColor c = dropOk_ ? QColor("#3fb950") : QColor("#f85149");
    p.setPen(QPen(c, 2));
    QColor fill = c;
    fill.setAlpha(45);
    p.setBrush(fill);
    p.drawRoundedRect(visualRect(dropIndex_).adjusted(1, 1, -1, -1), 4, 4);
}

void ProjectTreeView::contextMenuEvent(QContextMenuEvent* event) {
    const QModelIndex idx = indexAt(event->pos());
    Q_EMIT contextMenuRequestedFor(model_.refAt(idx), event->globalPos());
}

}  // namespace wpiemu::app::editor
