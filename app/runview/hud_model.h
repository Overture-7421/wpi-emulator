#pragma once

#include <QAbstractListModel>
#include <QSortFilterProxyModel>
#include <QString>

#include <vector>

#include "wpiemu/sdk/game/ui_descriptors.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::app::runview {

// One side of a Scoreboard widget.
struct HudSideState {
    QString label;
    QString score;
    QString status;           // empty when the status key is absent
    QString color;
    bool statusActive = false;
    bool leading = false;

    bool operator==(const HudSideState&) const = default;
};

// Evaluated, display-ready state of one game-plugin HUD widget descriptor.
struct HudItemState {
    QString id;
    QString title;
    sdk::HudWidgetKind kind = sdk::HudWidgetKind::Counter;
    sdk::HudAnchor anchor = sdk::HudAnchor::TopLeft;
    int order = 0;
    bool valid = false;      // false: the snapshot has no value for the key (shown as an em dash)
    bool active = false;     // Badge: on
    double fraction = 0.0;   // Meter / Gauge: 0..1
    QString text;            // value text (already formatted, with unit)
    QString color;           // "#rrggbb" or empty for the theme default
    // Scoreboard: sides[0] left; `text` is the centre caption, `lead` the margin text ("BLUE +6", "TIED"),
    // `leader` the index of the leading side or -1.
    std::vector<HudSideState> sides;
    QString lead;
    int leader = -1;

    bool operator==(const HudItemState&) const = default;
};

// Generic renderer logic: knows nothing about any game, only the descriptor + snapshot contract.
HudItemState evaluateHud(const sdk::HudWidgetDescriptor& descriptor, const sdk::ISimSnapshot* snapshot);

QString formatClock(double seconds);  // m:ss, negative clamped to 0

class HudModel : public QAbstractListModel {
    Q_OBJECT
public:
    enum Role {
        IdRole = Qt::UserRole + 1, TitleRole, KindRole, AnchorRole, OrderRole, ValidRole, ActiveRole,
        FractionRole, TextRole, ColorRole, SidesRole, LeadRole, LeaderRole
    };

    explicit HudModel(QObject* parent = nullptr);

    void setDescriptors(std::vector<sdk::HudWidgetDescriptor> descriptors);
    void update(const sdk::ISimSnapshot* snapshot);

    int rowCount(const QModelIndex& parent = {}) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    const HudItemState& item(int row) const { return states_[static_cast<std::size_t>(row)]; }
    // Items of one anchor corner/edge, sorted by descriptor order (used by the QML overlay).
    Q_INVOKABLE QObject* anchorModel(int anchor) const;
    static constexpr int kAnchorCount = 7;

private:
    std::vector<sdk::HudWidgetDescriptor> descriptors_;  // sorted by (anchor, order)
    std::vector<HudItemState> states_;
    std::vector<QSortFilterProxyModel*> proxies_;
};

}  // namespace wpiemu::app::runview
