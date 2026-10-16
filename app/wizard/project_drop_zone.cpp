#include "wizard/project_drop_zone.h"

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QStyle>
#include <QUrl>

#include "common/ui_helpers.h"

namespace wpiemu::app {

ProjectDropZone::ProjectDropZone(QWidget* parent) : QFrame(parent) {
    setAcceptDrops(true);
    setProperty("dropZone", true);
    setProperty("dragActive", false);
}

QString ProjectDropZone::pathFromMimeData(const QMimeData* mime) {
    if (mime == nullptr || !mime->hasUrls()) return {};
    for (const QUrl& url : mime->urls())
        if (url.isLocalFile()) return url.toLocalFile();
    return {};
}

void ProjectDropZone::setDragActive(bool active) {
    setProperty("dragActive", active);
    repolish(this);
}

void ProjectDropZone::dragEnterEvent(QDragEnterEvent* event) {
    if (!pathFromMimeData(event->mimeData()).isEmpty()) {
        event->acceptProposedAction();
        setDragActive(true);
    }
}

void ProjectDropZone::dragLeaveEvent(QDragLeaveEvent* event) {
    setDragActive(false);
    event->accept();
}

bool ProjectDropZone::acceptPayload(const QMimeData* mime) {
    const QString path = pathFromMimeData(mime);
    if (path.isEmpty()) return false;
    emit pathDropped(path);
    return true;
}

void ProjectDropZone::dropEvent(QDropEvent* event) {
    setDragActive(false);
    if (acceptPayload(event->mimeData())) event->acceptProposedAction();
}

}  // namespace wpiemu::app
