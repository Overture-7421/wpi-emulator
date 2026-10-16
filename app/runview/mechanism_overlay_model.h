#pragma once

#include <QAbstractListModel>
#include <QString>

#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::app::runview {

// One card per mechanism: id/label, "active" dot and its published values (positions, velocities...).
class MechanismOverlayModel : public QAbstractListModel {
    Q_OBJECT
public:
    enum Role { IdRole = Qt::UserRole + 1, LabelRole, TypeRole, ActiveRole, ValuesRole };

    explicit MechanismOverlayModel(QObject* parent = nullptr);

    void setConfig(std::shared_ptr<const sdk::RobotConfig> config) { config_ = std::move(config); }
    void update(const sdk::ISimSnapshot* snapshot);

    int rowCount(const QModelIndex& parent = {}) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    static QString formatValue(const QString& name, double value, const QString& unit);

private:
    struct Row {
        QString id, label, type, values;
        bool active = false;
        bool operator==(const Row&) const = default;
    };
    std::shared_ptr<const sdk::RobotConfig> config_;
    std::vector<Row> rows_;
};

}  // namespace wpiemu::app::runview
