#include "views/palette_panel.h"

#include <QMimeData>
#include <QVBoxLayout>

#include "model/device_directory.h"
#include "model/property_groups.h"
#include "model/tr.h"

namespace wpiemu::app::editor {

namespace {
constexpr int kTypeRole = Qt::UserRole + 1;
constexpr int kHeaderRole = Qt::UserRole + 2;
constexpr int kCategoryRole = Qt::UserRole + 3;
}  // namespace

QStringList TemplateList::mimeTypes() const { return {QString::fromLatin1(kTemplateMime)}; }

QMimeData* TemplateList::mimeData(const QList<QListWidgetItem*>& items) const {
    for (const QListWidgetItem* item : items) {
        if (item->data(kHeaderRole).toBool()) continue;
        auto* mime = new QMimeData;
        mime->setData(QString::fromLatin1(kTemplateMime), item->data(kTypeRole).toString().toUtf8());
        mime->setText(item->text());
        return mime;
    }
    return nullptr;
}

PalettePanel::PalettePanel(EditorController& controller, const IconFactory& icons, QWidget* parent)
    : QWidget(parent), controller_(controller), icons_(icons), filter_(new QLineEdit(this)), list_(new TemplateList(this)),
      add_(new QPushButton(this)) {
    auto* v = new QVBoxLayout(this);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(4);
    filter_->setPlaceholderText(ETR("Filter mechanism templates..."));
    filter_->setClearButtonEnabled(true);
    v->addWidget(filter_);
    list_->setDragEnabled(true);
    list_->setDragDropMode(QAbstractItemView::DragOnly);
    list_->setSelectionMode(QAbstractItemView::SingleSelection);
    list_->setFrameShape(QFrame::NoFrame);
    list_->setIconSize(QSize(18, 18));
    list_->setUniformItemSizes(false);
    list_->setSpacing(1);
    list_->setToolTip(ETR("Double-click to add. Drag onto a tree row to attach it there."));
    v->addWidget(list_, 1);
    v->addWidget(add_);

    connect(filter_, &QLineEdit::textChanged, this, [this](const QString& t) { applyFilter(t); });
    connect(list_, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem*) { addSelected(); });
    connect(list_, &QListWidget::itemActivated, this, [this](QListWidgetItem*) { addSelected(); });
    connect(list_, &QListWidget::currentItemChanged, this, [this] { updateButton(); });
    connect(add_, &QPushButton::clicked, this, [this] { addSelected(); });
    connect(&controller_.document(), &EditorDocument::catalogChanged, this, [this] { reload(); });
    connect(&controller_.selection(), &SelectionState::changed, this, [this] { updateButton(); });
    reload();
}

int PalettePanel::templateCount() const {
    int n = 0;
    for (int i = 0; i < list_->count(); ++i)
        if (!list_->item(i)->data(kHeaderRole).toBool()) ++n;
    return n;
}

void PalettePanel::reload() {
    list_->clear();
    const EditorCatalog& cat = controller_.document().catalog();
    std::vector<std::string> categories;
    for (const auto& t : cat.mechanismTemplates)
        if (std::find(categories.begin(), categories.end(), t.category) == categories.end()) categories.push_back(t.category);
    for (const auto& category : categories) {
        auto* header = new QListWidgetItem(QString::fromStdString(category.empty() ? "Other" : category).toUpper());
        header->setFlags(Qt::NoItemFlags);
        header->setData(kHeaderRole, true);
        QFont f = header->font();
        f.setBold(true);
        f.setPointSizeF(f.pointSizeF() * 0.8);
        header->setFont(f);
        header->setData(kCategoryRole, QString::fromStdString(category));
        list_->addItem(header);
        for (const auto& t : cat.mechanismTemplates) {
            if (t.category != category) continue;
            auto* item = new QListWidgetItem(icons_.glyph(QString::fromStdString(t.displayName).left(1).toUpper(), IconFactory::colorForCategory(QString::fromStdString(category))),
                                             QString::fromStdString(t.displayName));
            item->setData(kTypeRole, QString::fromStdString(t.type));
            item->setData(kCategoryRole, QString::fromStdString(category));
            QString tip = QString::fromStdString(t.description);
            if (t.origin != "core") tip += QStringLiteral("\n[%1]").arg(QString::fromStdString(t.origin));
            item->setToolTip(tip);
            item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsDragEnabled);
            list_->addItem(item);
        }
    }
    applyFilter(filter_->text());
    updateButton();
}

void PalettePanel::applyFilter(const QString& text) {
    QString currentCategory;
    std::vector<QListWidgetItem*> pendingHeader;
    for (int i = 0; i < list_->count(); ++i) {
        QListWidgetItem* item = list_->item(i);
        if (item->data(kHeaderRole).toBool()) {
            item->setHidden(true);
            pendingHeader = {item};
            continue;
        }
        const bool match = text.isEmpty() || item->text().contains(text, Qt::CaseInsensitive) ||
                           item->data(kCategoryRole).toString().contains(text, Qt::CaseInsensitive);
        item->setHidden(!match);
        if (match && !pendingHeader.empty()) pendingHeader.front()->setHidden(false);
    }
}

void PalettePanel::updateButton() {
    const SelectionRef target = formTargetOf(controller_.document().config(), controller_.selection().current());
    const bool attach = target.kind == SelKind::Mechanism && controller_.selection().current().kind != SelKind::Slot;
    const QListWidgetItem* item = list_->currentItem();
    const bool ok = item != nullptr && !item->data(kHeaderRole).toBool();
    add_->setEnabled(ok);
    const QString name = ok ? item->text() : QString();
    if (!ok) add_->setText(ETR("Select a template"));
    else if (attach) add_->setText(ETR("Add %1 to the selected mechanism").arg(name));
    else add_->setText(ETR("Add %1 to the robot").arg(name));
}

void PalettePanel::addSelected() {
    const QListWidgetItem* item = list_->currentItem();
    if (item == nullptr || item->data(kHeaderRole).toBool()) return;
    controller_.addMechanismToSelection(item->data(kTypeRole).toString());
}

}  // namespace wpiemu::app::editor
