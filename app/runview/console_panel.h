#pragma once

#include <QWidget>

#include "console_model.h"

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QListView;
class QToolButton;

namespace wpiemu::app::runview {

// Bottom console: robot stdout/stderr and emulator messages with level / stream filters, search, copy.
class ConsolePanel : public QWidget {
    Q_OBJECT
public:
    explicit ConsolePanel(QWidget* parent = nullptr);

    ConsoleModel& model() { return model_; }
    ConsoleFilterModel& filter() { return filter_; }

    void append(const RunLogEntry& entry);
    void clear();
    QString visibleText() const { return filter_.plainText(); }
    int visibleLineCount() const { return filter_.rowCount(); }
    int totalLineCount() const { return model_.rowCount(); }

    void setFollowTail(bool follow);

public slots:
    void copy();   // selected lines, or every visible line when nothing is selected

private:
    void updateCount();
    void scrollToTailIfFollowing();

    ConsoleModel model_;
    ConsoleFilterModel filter_;
    QListView* view_;
    QComboBox* levelCombo_;
    QLineEdit* search_;
    QLabel* count_;
    QToolButton* streams_;
    QToolButton* follow_button_ = nullptr;
    bool follow_ = true;
    bool scrollQueued_ = false;
};

}  // namespace wpiemu::app::runview
