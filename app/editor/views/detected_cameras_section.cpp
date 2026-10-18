#include "views/detected_cameras_section.h"

#include <QGridLayout>
#include <QPushButton>

#include "model/config_ops.h"
#include "model/tr.h"
#include "wpiemu/sdk/config/value_codec.h"

namespace wpiemu::app::editor {

namespace {

QString qs(const std::string& s) { return QString::fromStdString(s); }

QLabel* mutedLabel(const QString& text, QWidget* parent) {
    auto* l = new QLabel(text, parent);
    l->setWordWrap(true);
    l->setEnabled(false);
    return l;
}

}  // namespace

DetectedCamerasSection::DetectedCamerasSection(EditorController& controller, const IconFactory& icons, QWidget* parent)
    : QFrame(parent), controller_(controller), icons_(icons), title_(new QLabel(this)), rows_(new QVBoxLayout) {
    setObjectName(QStringLiteral("cameras.section"));
    auto* v = new QVBoxLayout(this);
    v->setContentsMargins(8, 0, 8, 4);
    v->setSpacing(2);
    auto* line = new QFrame(this);
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    v->addWidget(line);
    title_->setObjectName(QStringLiteral("cameras.title"));
    title_->setToolTip(ETR("The PhotonVision and Limelight cameras the robot code uses that the project does not list yet."));
    QFont f = title_->font();
    f.setBold(true);
    title_->setFont(f);
    v->addWidget(title_);
    rows_->setContentsMargins(0, 0, 0, 0);
    rows_->setSpacing(6);
    v->addLayout(rows_);

    setVisible(false);
    connect(&controller_, &EditorController::detectedCamerasUpdated, this, [this] { refresh(); });
    connect(&controller_.document(), &EditorDocument::changed, this, [this] { refresh(); });
    refresh();
}

void DetectedCamerasSection::refresh() {
    const sdk::RobotConfig& config = controller_.document().config();
    const std::vector<UnconfiguredCamera> rows = unconfiguredCameras(config, controller_.detectedCameras());

    // Rebuild only when what the rows show changed: the document signals every edit, also a drag at frame rate.
    QString signature;
    for (const UnconfiguredCamera& row : rows) {
        signature += qs(ops::vendorDisplayName(row.seen.vendor)) + '|' + qs(row.seen.ntName) + '|' + detectedCameraStatusText(row.seen);
        for (const std::string& id : row.renameCandidates)
            if (const sdk::CameraConfig* cam = config.findCamera(id))
                signature += '|' + qs(id) + '=' + qs(ops::cameraDisplayName(*cam)) + '/' + qs(cam->networkName());
        signature += '\n';
    }
    if (signature == shown_) return;
    shown_ = signature;

    while (QLayoutItem* item = rows_->takeAt(0)) {
        if (QWidget* w = item->widget()) {
            w->setParent(nullptr);
            w->deleteLater();
        }
        delete item;
    }
    for (const UnconfiguredCamera& row : rows) rows_->addWidget(makeRow(row));
    rowCount_ = static_cast<int>(rows.size());
    title_->setText(ETR("Cameras (%1)").arg(rowCount_));
    setVisible(rowCount_ > 0);
}

QWidget* DetectedCamerasSection::makeRow(const UnconfiguredCamera& row) {
    const sdk::RobotConfig& config = controller_.document().config();
    const sdk::CameraRobotState& seen = row.seen;
    const QString name = qs(seen.ntName);
    const QString vendor = qs(ops::vendorDisplayName(seen.vendor));
    const QString key = vendor + ':' + name;

    auto* w = new QFrame(this);
    w->setObjectName(QStringLiteral("cameras.row:") + key);
    auto* g = new QGridLayout(w);
    g->setContentsMargins(0, 0, 0, 0);
    g->setHorizontalSpacing(6);
    g->setVerticalSpacing(1);
    g->setColumnStretch(1, 1);

    auto* icon = new QLabel(w);
    icon->setPixmap(icons_.node(NodeKind::Camera).pixmap(16, 16));
    g->addWidget(icon, 0, 0, Qt::AlignTop);
    auto* title = new QLabel(name, w);
    QFont f = title->font();
    f.setBold(true);
    title->setFont(f);
    title->setObjectName(QStringLiteral("cameras.name:") + key);
    g->addWidget(title, 0, 1);

    auto* add = new QPushButton(ETR("Add camera"), w);
    add->setObjectName(QStringLiteral("cameras.add:") + key);
    add->setCursor(Qt::PointingHandCursor);
    add->setToolTip(ETR("Add a %1 camera called \"%2\" to the project. It starts at the front of the robot: move it to where it sits on the real robot.")
                        .arg(vendor, name));
    g->addWidget(add, 0, 2);
    connect(add, &QPushButton::clicked, this, [this, seen] { controller_.addCameraFromDetected(seen); });

    auto* status = mutedLabel(vendor + QStringLiteral(" \u00b7 ") + detectedCameraStatusText(seen), w);
    status->setObjectName(QStringLiteral("cameras.status:") + key);
    g->addWidget(status, 1, 1, 1, 2);

    // The robot code's name differs from the name of a camera of the project that the code does not use: a typo or a renamed camera.
    QStringList unused;
    std::string only;
    for (const std::string& id : row.renameCandidates)
        if (const sdk::CameraConfig* cam = config.findCamera(id)) {
            unused << ETR("%1 (%2)").arg(qs(ops::cameraDisplayName(*cam)), qs(cam->networkName()));
            only = id;
        }
    if (!unused.isEmpty()) {
        const bool one = unused.size() == 1;
        auto* hint = mutedLabel(one ? ETR("Not used by the robot code: %1. The same camera?").arg(unused.front())
                                    : ETR("Not used by the robot code: %1. If one of them is this camera, set its NetworkTables name to \"%2\".")
                                          .arg(unused.join(QStringLiteral(", ")), name),
                                w);
        hint->setObjectName(QStringLiteral("cameras.hint:") + key);
        g->addWidget(hint, 2, 1, 1, 2);
        if (one) {
            auto* rename = new QPushButton(ETR("Use this name"), w);
            rename->setObjectName(QStringLiteral("cameras.rename:") + key);
            rename->setCursor(Qt::PointingHandCursor);
            rename->setToolTip(ETR("Set the NetworkTables name of that camera to \"%1\", the name the robot code uses.").arg(name));
            g->addWidget(rename, 3, 1, 1, 1, Qt::AlignLeft);
            connect(rename, &QPushButton::clicked, this, [this, only, ntName = seen.ntName] {
                const SelectionRef camera = SelectionRef::camera(only);
                if (controller_.document().setProperty(camera, "camera.general", "ntName", sdk::Value(ntName))) controller_.selection().set(camera);
            });
        }
    }
    return w;
}

}  // namespace wpiemu::app::editor
