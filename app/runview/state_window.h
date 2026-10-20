#pragma once

#include <QWidget>

#include "state_model.h"

class QLabel;
class QTreeView;

namespace wpiemu::app::runview {

// Separate, non-modal window with the live tree of the robot's state (mechanisms, sensors, zones, cameras).
class StateWindow : public QWidget {
    Q_OBJECT
public:
    explicit StateWindow(const ViewportTheme& theme, QWidget* parent = nullptr);

    StateModel* model() const { return model_; }
    QTreeView* tree() const { return tree_; }
    QLabel* emptyLabel() const { return empty_; }
    QLabel* footerLabel() const { return footer_; }

    // Applies a snapshot (nullptr = no simulation running) and refreshes the empty-state text and the clock.
    void showSnapshot(const sdk::ISimSnapshot* snapshot);

signals:
    // The user closed the window (not emitted for programmatic hide()).
    void closed();

protected:
    void closeEvent(QCloseEvent*) override;

private:
    void refreshEmptyState(bool haveSnapshot);

    StateModel* model_;
    QTreeView* tree_;
    QLabel* empty_;
    QLabel* footer_;
};

}  // namespace wpiemu::app::runview
