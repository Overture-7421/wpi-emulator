#pragma once

#include <QFrame>

class QMimeData;

namespace wpiemu::app {

// A dashed target that accepts a dragged folder (or any file inside a project) and reports its path.
class ProjectDropZone : public QFrame {
    Q_OBJECT
public:
    explicit ProjectDropZone(QWidget* parent = nullptr);

    // First local path in the drop payload, empty when there is none (exposed for tests).
    static QString pathFromMimeData(const QMimeData* mime);
    // Reports the path in `mime`; returns whether something usable was dropped.
    bool acceptPayload(const QMimeData* mime);

signals:
    void pathDropped(const QString& path);

protected:
    void dragEnterEvent(QDragEnterEvent* event) override;
    void dragLeaveEvent(QDragLeaveEvent* event) override;
    void dropEvent(QDropEvent* event) override;

private:
    void setDragActive(bool active);
};

}  // namespace wpiemu::app
