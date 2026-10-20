#pragma once

#include <QAbstractItemModel>
#include <QColor>
#include <QHash>
#include <QIcon>
#include <QString>

#include <memory>
#include <vector>

#include "run_backend.h"
#include "scene_model.h"

namespace wpiemu::app::runview {

// Live tree of the robot's state for the "Robot state" window: Mechanisms, Sensors, Zones and Cameras, each an
// expandable row. Columns are Name | Value. Rows are stable between snapshots (only changed cells emit
// dataChanged); a different set of rows resets the model.
class StateModel : public QAbstractItemModel {
    Q_OBJECT
public:
    enum Role {
        KeyRole = Qt::UserRole + 1,   // stable key: "mechanism:<id>", "sensor:<id>", ... / "<parent>/<field>"
        ToneRole,                     // Tone as int
    };
    enum class Tone { Normal, Muted, Good, Warning };

    explicit StateModel(const ViewportTheme& theme, QObject* parent = nullptr);
    ~StateModel() override;

    void setScene(const RunSceneInfo& info);
    // Rebuilds the tree from the snapshot (labels/kinds/units come from setScene); nullptr empties it.
    void update(const sdk::ISimSnapshot* snapshot);

    bool isEmpty() const;
    QModelIndex findKey(const QString& key) const;

    int rowCount(const QModelIndex& parent = {}) const override;
    int columnCount(const QModelIndex& = {}) const override { return 2; }
    QModelIndex index(int row, int column, const QModelIndex& parent = {}) const override;
    QModelIndex parent(const QModelIndex& child) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

    // Pure formatting helpers, exposed for tests.
    static QString formatNumber(double value, const QString& unit);
    static QString unitSymbol(sdk::PropertyUnit unit);

private:
    struct Node {
        QString key;
        QString name;
        QString value;
        QString tooltip;
        Tone dot = Tone::Normal;
        bool hasDot = false;
        Tone tone = Tone::Normal;
        Node* parent = nullptr;
        int row = 0;
        std::vector<Node> children;
    };

    Node build(const sdk::ISimSnapshot* snapshot) const;
    static void link(Node& node);
    static bool sameShape(const Node& a, const Node& b);
    void merge(Node& current, Node&& next, const QModelIndex& index);
    QColor color(Tone tone) const;
    const Node* nodeAt(const QModelIndex& index) const;
    QModelIndex indexOf(const Node* node, int column = 0) const;
    static const Node* find(const Node& node, const QString& key);

    const ViewportTheme& theme_;
    RunSceneInfo info_;
    Node root_;
    mutable QHash<QRgb, QIcon> dots_;
};

}  // namespace wpiemu::app::runview
