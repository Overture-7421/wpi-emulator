#pragma once

// Renders one PropertyGroup (schema + values) as a form: grouped rows, units, conditional visibility,
// per-field issue badges. It edits nothing itself: it reports user edits.

#include <QFormLayout>
#include <QLabel>
#include <QMap>
#include <QWidget>

#include <vector>

#include "model/property_groups.h"
#include "model/units.h"
#include "views/field_editors.h"

namespace wpiemu::app::editor {

class PropertyFormWidget : public QWidget {
    Q_OBJECT
public:
    PropertyFormWidget(const PropertyGroup& group, UnitSystem units, QWidget* parent = nullptr);

    const std::string& groupId() const { return groupId_; }
    // Pushes new values into the existing editors (no rebuild, keeps focus and scroll).
    void setValues(const sdk::Value& values, bool authoritative = true);
    // key -> (severity, message); keys without an entry are cleared.
    void setFieldIssues(const QMap<QString, QPair<int, QString>>& issues);
    // Focuses and briefly highlights a field; false when the key is not in this form.
    bool focusField(const QString& key);
    bool hasField(const QString& key) const;
    FieldEditor* editorFor(const QString& key) const;
    int visibleRowCount() const;

Q_SIGNALS:
    void valueEdited(const QString& groupId, const QString& key, const wpiemu::sdk::Value& value);

private:
    struct Row {
        sdk::PropertySpec spec;
        FieldEditor* editor = nullptr;
        QWidget* container = nullptr;
        QWidget* label = nullptr;
        QLabel* badge = nullptr;
        bool stacked = false;   // composite editors get a full-width row under their label
    };
    void updateVisibility();
    bool conditionMet(const sdk::PropertySpec& spec) const;

    std::string groupId_;
    sdk::Value values_;
    QString editingKey_;
    QFormLayout* form_ = nullptr;
    std::vector<Row> rows_;
};

}  // namespace wpiemu::app::editor
