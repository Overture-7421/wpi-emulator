#pragma once

// The camera cards and vision notices of the run view (ADR 0017): per camera a state chip, the
// capture rate measured from the snapshot's frame counter, the latest latency, the tags in view and
// what the robot code asked the camera for. Presentation logic evaluated from the snapshot like
// hud_model.h; the QML cards only show the strings and the tone colour.

#include <QAbstractListModel>
#include <QDateTime>
#include <QHash>
#include <QString>
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "camera_rig.h"
#include "run_control.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/severity.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::app::runview {

// The tone (chip and 3D colour) of a camera state: ok green, waiting blue, disabled grey, problem
// red.
Tone toneOf(CameraState state);

// CameraSample::status ("" = fine) in words; an unknown status is shown as it is.
QString cameraStatusText(const std::string& status);

// Capture rate from a frame counter and the sim clock: the average over the last ~3 s of sim time
// (steady at 30 Hz although captures land on ticks), nothing until a second has been seen. A clock
// or a counter that goes backwards (a reset) starts over.
class RateMeter {
public:
    void reset();
    void add(std::uint64_t count, std::int64_t simTimeUs);
    std::optional<double> hz() const { return hz_; }

private:
    struct Sample {
        std::int64_t timeUs = 0;
        std::uint64_t count = 0;
    };
    static constexpr std::size_t kSamples = 16;  // one every >= 250 ms
    std::array<Sample, kSamples> ring_{};
    std::size_t head_ = 0;  // next slot to write
    std::size_t size_ = 0;
    std::optional<double> hz_;
};

// Display-ready state of one camera.
struct CameraCardState {
    QString id;
    QString label;
    QString vendor;  // "PhotonVision" / "Limelight"
    CameraState state = CameraState::Waiting;
    QString stateText;    // Connected / Waiting for robot / Disabled / the status in words
    QString rateText;     // "30 Hz" or an em dash
    QString latencyText;  // "25 ms" or an em dash
    QString metricsText;  // "30 Hz · 25 ms", or "Not capturing" for a disabled camera
    int visibleCount = 0;
    QString tagsText;     // "3 tags: 1, 2, 12" / "No tags in view"
    QString requestText;  // what the robot code asked for (pipeline, driver mode, LED), or empty
    bool seesTags = false;

    bool operator==(const CameraCardState&) const = default;
};

// `robot`: what the robot process reported for this camera (CameraRobotState), nullptr when it
// reported nothing.
CameraCardState evaluateCameraCard(const sdk::CameraSample& camera,
                                   const sdk::CameraRobotState* robot,
                                   std::optional<double> rateHz);

// The detected-camera entry of `camera`: the one the bridge matched by id, else (a state without an
// id) the one with the camera's NetworkTables name and vendor. `networkName` empty = unknown, only
// the id matches.
const sdk::CameraRobotState* matchRobotState(const sdk::CameraSample& camera,
                                             const std::string& networkName,
                                             std::span<const sdk::CameraRobotState> detected);

// A vision problem the runtime raised as a HubWarning (vision_layout_missing,
// vision_camera_unconfigured, vision_camera_status).
struct VisionNotice {
    QString code;
    QString title;   // readable headline for the code
    QString detail;  // the warning's own message
    sdk::Severity severity = sdk::Severity::Warning;

    bool operator==(const VisionNotice&) const = default;
};

// Headline for a vision HubWarning code; empty for any other code.
QString visionNoticeTitle(const std::string& code);
// The vision warnings of `snapshot` (other warnings are not the run view's business here).
std::vector<VisionNotice> visionNotices(const sdk::ISimSnapshot* snapshot);

class VisionNoticeModel : public QAbstractListModel {
    Q_OBJECT
public:
    enum Role { TitleRole = Qt::UserRole + 1, DetailRole, SeverityRole, ToneRole };
    // The same notice is announced (noticeRaised) again after it went away at the earliest this
    // long after it was last announced, so a problem that flickers does not flood the console.
    static constexpr qint64 kRepeatAnnounceMs = 10000;

    explicit VisionNoticeModel(QObject* parent = nullptr);
    // `nowMs` (wall clock) only rate-limits noticeRaised.
    void update(const sdk::ISimSnapshot* snapshot,
                qint64 nowMs = QDateTime::currentMSecsSinceEpoch());
    const std::vector<VisionNotice>& notices() const { return notices_; }

    int rowCount(const QModelIndex& parent = {}) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

signals:
    // A notice that was not shown before (severity: 0 info, 1 warning, 2 error), e.g. for the
    // console.
    void noticeRaised(const QString& title, const QString& detail, int severity);

private:
    std::vector<VisionNotice> notices_;
    QHash<QString, qint64> announcedMs_;  // "code|detail" -> when it was last announced
};

class CameraHudModel : public QAbstractListModel {
    Q_OBJECT
    Q_PROPERTY(QObject* notices READ notices CONSTANT)
public:
    enum Role {
        IdRole = Qt::UserRole + 1,
        LabelRole,
        VendorRole,
        StateRole,
        StateTextRole,
        ToneRole,
        RateRole,
        LatencyRole,
        MetricsRole,
        TagsRole,
        RequestRole,
        ActiveRole
    };

    explicit CameraHudModel(QObject* parent = nullptr);

    // The project's cameras give the NetworkTables names the detected-camera states are matched
    // with.
    void setConfig(std::shared_ptr<const sdk::RobotConfig> config) { config_ = std::move(config); }
    void update(const sdk::ISimSnapshot* snapshot);
    const CameraCardState& card(int row) const { return cards_[static_cast<std::size_t>(row)]; }
    QObject* notices() { return &notices_; }
    VisionNoticeModel& noticeModel() { return notices_; }

    int rowCount(const QModelIndex& parent = {}) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

private:
    std::shared_ptr<const sdk::RobotConfig> config_;
    std::vector<CameraCardState> cards_;
    std::vector<std::string> ids_;  // camera id of each card
    std::vector<RateMeter> meters_;
    VisionNoticeModel notices_;
};

}  // namespace wpiemu::app::runview
