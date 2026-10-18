#pragma once

// Body of the inspector's Activation card for a zone: "Active when" ALL / ANY of a list of conditions on
// values mechanisms publish (sdk::ZoneActivation). An empty list means the zone is always active. One row
// per condition: mechanism, published value name, Above / Below / Within, threshold (+ tolerance for Within),
// absolute-value toggle and remove. Every edit is one undoable document edit (EditorDocument::setZone*).
//
// Object names for tests (I = condition index): "activation.mode" (QComboBox), "activation.empty" (QLabel),
// "activation.add" (QToolButton), and per row "activation.row:I" (QFrame), "activation.mechanism:I" (QComboBox),
// "activation.value:I" (QLineEdit), "activation.op:I" (QComboBox), "activation.threshold:I" /
// "activation.tolerance:I" (QDoubleSpinBox), "activation.absolute:I" (QCheckBox), "activation.remove:I"
// (QToolButton), "activation.badge:I" (QLabel).

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QLabel>
#include <QLineEdit>
#include <QMap>
#include <QToolButton>
#include <QWidget>

#include <string>
#include <vector>

#include "viewmodel/editor_controller.h"

namespace wpiemu::app::editor {

class ActivationCard : public QWidget {
    Q_OBJECT
public:
    ActivationCard(EditorController& controller, const std::string& zoneId, QWidget* parent = nullptr);

    // Re-reads the zone from the document: values are updated in place (a focused editor keeps its text unless
    // the change is authoritative: undo / redo / load); rows are rebuilt only when the condition count changes.
    void refresh(bool authoritative);
    // key "activation:<index>" -> (severity, message), as produced by locateIssue; other keys are ignored.
    void setIssues(const QMap<QString, QPair<int, QString>>& issues);
    // Focuses the value field of a condition row; false when the row does not exist.
    bool focusCondition(int index);
    int conditionCount() const { return static_cast<int>(rows_.size()); }

private:
    struct Row {
        QFrame* frame = nullptr;
        QComboBox* mechanism = nullptr;
        QLineEdit* value = nullptr;
        QComboBox* op = nullptr;
        QDoubleSpinBox* threshold = nullptr;
        QDoubleSpinBox* tolerance = nullptr;
        QCheckBox* absolute = nullptr;
        QToolButton* remove = nullptr;
        QLabel* badge = nullptr;
    };

    const sdk::ZoneConfig* zone() const;
    void rebuildRows();
    Row makeRow(int index);
    void syncRow(Row& row, const sdk::ZoneCondition& condition, bool authoritative);
    void syncMechanisms(Row& row, const std::string& selected);
    sdk::ZoneCondition conditionOf(const Row& row) const;
    void commit(int index);

    EditorController& controller_;
    std::string zoneId_;
    QComboBox* mode_ = nullptr;
    QLabel* empty_ = nullptr;
    QWidget* rowsHost_ = nullptr;
    QLayout* rowsLayout_ = nullptr;
    std::vector<Row> rows_;
    QMap<QString, QPair<int, QString>> issues_;
};

}  // namespace wpiemu::app::editor
