#pragma once

// Mechanism template palette: the templates the catalog offers (core + plugin), grouped by category.
// Double-click / Enter / the Add button attach one to the selection (or the chassis); dragging a
// template onto a tree row attaches it there.

#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QWidget>

#include "viewmodel/editor_controller.h"
#include "views/icon_factory.h"

namespace wpiemu::app::editor {

class TemplateList : public QListWidget {
    Q_OBJECT
public:
    explicit TemplateList(QWidget* parent = nullptr) : QListWidget(parent) {}

protected:
    QMimeData* mimeData(const QList<QListWidgetItem*>& items) const override;
    QStringList mimeTypes() const override;
};

class PalettePanel : public QWidget {
    Q_OBJECT
public:
    PalettePanel(EditorController& controller, const IconFactory& icons, QWidget* parent = nullptr);

    void reload();
    TemplateList* list() const { return list_; }
    int templateCount() const;

private:
    void applyFilter(const QString& text);
    void addSelected();
    void updateButton();

    EditorController& controller_;
    const IconFactory& icons_;
    QLineEdit* filter_;
    TemplateList* list_;
    QPushButton* add_;
};

}  // namespace wpiemu::app::editor
