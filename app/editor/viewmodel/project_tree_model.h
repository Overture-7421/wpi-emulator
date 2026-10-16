#pragma once

// The project tree: robot > chassis (COM, shapes), drivetrain > wheels > slots, mechanisms (nested by
// attachment) > slots / shapes / volumes, sensors, game plugin. Each row carries a SelectionRef, a
// kind and (for slots) the mapping state; the delegate paints them. Accepts device / template /
// mechanism drops and reports them as signals, never edits the document itself.

#include <QAbstractItemModel>
#include <QString>

#include <memory>
#include <vector>

#include "model/device_directory.h"
#include "model/editor_document.h"
#include "model/selection.h"
#include "model/slot_status.h"

namespace wpiemu::app::editor {

enum class NodeKind { Robot, Chassis, Com, Drivetrain, Wheel, Slot, Shape, Mechanism, Group, Volume, Plugin };

class ProjectTreeModel : public QAbstractItemModel {
    Q_OBJECT
public:
    enum Role {
        KindRole = Qt::UserRole + 1,   // NodeKind
        StateRole,                     // SlotState for slots, -1 otherwise
        DetailRole,                    // secondary text (mapped device, link, ...)
        KeyRole,                       // stable node key (expansion state)
        WarningRole,                   // bool: a mechanism / wheel with unfilled required slots below it
        RefKeyRole,                    // SelectionRef::key()
    };

    ProjectTreeModel(EditorDocument& document, const DeviceDirectory& devices, QObject* parent = nullptr);

    void refresh();
    SelectionRef refAt(const QModelIndex& index) const;
    QModelIndex indexFor(const SelectionRef& ref) const;
    QString keyAt(const QModelIndex& index) const;
    QModelIndex indexForKey(const QString& key) const;
    // Parent mechanism for a drop target index (chassis when none).
    sdk::MechanismAttachment attachmentFor(const QModelIndex& index) const;

    // ---- QAbstractItemModel ----
    QModelIndex index(int row, int column, const QModelIndex& parent = {}) const override;
    QModelIndex parent(const QModelIndex& child) const override;
    int rowCount(const QModelIndex& parent = {}) const override;
    int columnCount(const QModelIndex& = {}) const override { return 1; }
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;
    QStringList mimeTypes() const override;
    QMimeData* mimeData(const QModelIndexList& indexes) const override;
    Qt::DropActions supportedDropActions() const override { return Qt::CopyAction | Qt::MoveAction; }
    bool canDropMimeData(const QMimeData* data, Qt::DropAction action, int row, int column, const QModelIndex& parent) const override;
    bool dropMimeData(const QMimeData* data, Qt::DropAction action, int row, int column, const QModelIndex& parent) override;

Q_SIGNALS:
    void deviceDropped(const wpiemu::app::editor::SelectionRef& target, const wpiemu::sdk::DeviceKey& device, quint32 capabilityBits);
    void templateDropped(const QString& type, const wpiemu::sdk::MechanismAttachment& attachment);
    void mechanismDropped(const QString& id, const wpiemu::sdk::MechanismAttachment& attachment);

private:
    struct Node {
        NodeKind kind = NodeKind::Group;
        SelectionRef ref;
        QString key;
        QString text;
        QString detail;
        QString tooltip;
        int state = -1;
        bool warning = false;
        Node* parent = nullptr;
        std::vector<std::unique_ptr<Node>> children;
    };

    std::unique_ptr<Node> build() const;
    void addMechanismNode(Node& parentNode, const sdk::RobotConfig& c, const sdk::MechanismConfig& m) const;
    static void flatten(const Node& n, std::vector<const Node*>& out);
    static bool sameShape(const Node& a, const Node& b);
    static void copyContents(Node& dst, const Node& src);
    const Node* nodeAt(const QModelIndex& index) const;
    QModelIndex indexOfNode(const Node* node) const;
    const Node* findByKey(const Node& root, const QString& key) const;
    const Node* findByRef(const Node& root, const SelectionRef& ref) const;
    bool acceptsDevice(const Node* target, sdk::CapabilitySet caps) const;

    EditorDocument& document_;
    const DeviceDirectory& devices_;
    std::unique_ptr<Node> root_;   // invisible root; first child is the robot
};

}  // namespace wpiemu::app::editor

Q_DECLARE_METATYPE(wpiemu::sdk::DeviceKey)
Q_DECLARE_METATYPE(wpiemu::sdk::MechanismAttachment)
