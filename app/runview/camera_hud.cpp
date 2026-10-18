#include "camera_hud.h"

#include <QCoreApplication>
#include <QStringList>
#include <algorithm>
#include <cmath>

#include "status_chip.h"
#include "wpiemu/sdk/config/validation_codes.h"

namespace wpiemu::app::runview {
namespace {

QString tr(const char* s) {
    return QCoreApplication::translate("CameraHud", s);
}

constexpr std::int64_t kSampleSpacingUs = 250000;
constexpr std::int64_t kWindowUs = 3000000;
constexpr std::int64_t kMinSpanUs = 1000000;
constexpr std::size_t kMaxListedTags = 8;

QString dash() {
    return QStringLiteral("—");
}

QString formatRate(double hz) {
    return (hz >= 10.0 ? QString::number(static_cast<int>(std::lround(hz)))
                       : QString::number(hz, 'f', 1)) +
           QStringLiteral(" Hz");
}

QString tagsText(const std::vector<int>& ids) {
    if (ids.empty()) return tr("No tags in view");
    std::vector<int> sorted = ids;
    std::sort(sorted.begin(), sorted.end());
    QStringList listed;
    for (std::size_t i = 0; i < sorted.size() && i < kMaxListedTags; ++i)
        listed << QString::number(sorted[i]);
    QString list = listed.join(QStringLiteral(", "));
    if (sorted.size() > kMaxListedTags) list += QStringLiteral(", …");
    return sorted.size() == 1 ? tr("1 tag: %1").arg(list)
                              : tr("%1 tags: %2").arg(sorted.size()).arg(list);
}

// The LED values differ per vendor: PhotonVision VisionLEDMode (-1 default, 0 off, 1 on, 2 blink)
// and Limelight ledMode (0 pipeline default, 1 off, 2 blink, 3 on).
QString ledText(sdk::CameraVendor vendor, int mode) {
    if (vendor == sdk::CameraVendor::PhotonVision) {
        switch (mode) {
            case -1:
                return {};
            case 0:
                return tr("LED off");
            case 1:
                return tr("LED on");
            case 2:
                return tr("LED blinking");
            default:
                break;
        }
    } else {
        switch (mode) {
            case 0:
                return {};
            case 1:
                return tr("LED off");
            case 2:
                return tr("LED blinking");
            case 3:
                return tr("LED on");
            default:
                break;
        }
    }
    return tr("LED mode %1").arg(mode);
}

QString requestText(sdk::CameraVendor vendor, const sdk::CameraRobotState* robot) {
    if (robot == nullptr) return {};
    if (!robot->present) return tr("The robot code does not use this camera");
    QStringList parts;
    if (robot->pipelineIndex) parts << tr("pipeline %1").arg(*robot->pipelineIndex);
    if (robot->driverMode.value_or(false)) parts << tr("driver mode");
    if (robot->ledMode)
        if (const QString led = ledText(vendor, *robot->ledMode); !led.isEmpty()) parts << led;
    if (parts.isEmpty()) return tr("The robot code uses this camera");
    return tr("Robot asked for: %1").arg(parts.join(QStringLiteral(" · ")));
}

bool isVisionCode(const std::string& code) {
    return code.rfind("vision_", 0) == 0;
}

QString defaultDetail(const std::string& code) {
    namespace vc = sdk::validation_code;
    if (code == vc::kVisionLayoutMissing) return tr("No AprilTags can be drawn or detected.");
    if (code == vc::kVisionCameraUnconfigured)
        return tr("The robot code uses a camera that this project does not configure.");
    if (code == vc::kVisionCameraStatus) return tr("A camera reported a problem.");
    return {};
}

}  // namespace

Tone toneOf(CameraState state) {
    switch (state) {
        case CameraState::Ok:
            return Tone::Success;
        case CameraState::Waiting:
            return Tone::Info;
        case CameraState::Disabled:
            return Tone::Neutral;
        case CameraState::Problem:
            break;
    }
    return Tone::Error;
}

QString cameraStatusText(const std::string& status) {
    if (status.empty()) return {};
    if (status == "disabled") return tr("Disabled");
    if (status == "photon_uuid_unsupported") return tr("PhotonLib version not supported");
    if (status == "foreign_publisher") return tr("Published by the robot code");
    if (status == "ntcore_unavailable") return tr("NetworkTables not available");
    return QString::fromStdString(status);
}

// ---- RateMeter
// ------------------------------------------------------------------------------------
void RateMeter::reset() {
    head_ = 0;
    size_ = 0;
    hz_.reset();
}

void RateMeter::add(std::uint64_t count, std::int64_t simTimeUs) {
    const auto newest = [this]() -> const Sample& {
        return ring_[(head_ + kSamples - 1) % kSamples];
    };
    if (size_ > 0 && (simTimeUs < newest().timeUs || count < newest().count)) reset();
    if (size_ == 0 || simTimeUs - newest().timeUs >= kSampleSpacingUs) {
        ring_[head_] = {simTimeUs, count};
        head_ = (head_ + 1) % kSamples;
        size_ = std::min(size_ + 1, kSamples);
    }
    // The oldest sample that is not older than the window.
    for (std::size_t i = 0; i < size_; ++i) {
        const Sample& s = ring_[(head_ + kSamples - size_ + i) % kSamples];
        const std::int64_t spanUs = simTimeUs - s.timeUs;
        if (spanUs > kWindowUs) continue;
        if (spanUs >= kMinSpanUs)
            hz_ = static_cast<double>(count - s.count) * 1e6 / static_cast<double>(spanUs);
        break;
    }
}

// ---- cards
// ------------------------------------------------------------------------------------------
CameraCardState evaluateCameraCard(const sdk::CameraSample& camera,
                                   const sdk::CameraRobotState* robot,
                                   std::optional<double> rateHz) {
    CameraCardState s;
    s.id = QString::fromStdString(camera.id);
    s.label = QString::fromStdString(camera.label.empty() ? camera.id : camera.label);
    s.vendor = camera.vendor == sdk::CameraVendor::Limelight ? QStringLiteral("Limelight")
                                                             : QStringLiteral("PhotonVision");
    s.state = cameraStateOf(camera);
    switch (s.state) {
        case CameraState::Ok:
            s.stateText = tr("Connected");
            break;
        case CameraState::Waiting:
            s.stateText = tr("Waiting for robot");
            break;
        case CameraState::Disabled:
            s.stateText = tr("Disabled");
            break;
        case CameraState::Problem:
            s.stateText = cameraStatusText(camera.status);
            break;
    }
    const bool capturing = s.state != CameraState::Disabled;
    s.rateText = capturing && rateHz ? formatRate(*rateHz) : dash();
    s.latencyText = capturing && camera.lastLatencyMs > 0.0
                        ? QString::number(camera.lastLatencyMs, 'f', 0) + QStringLiteral(" ms")
                        : dash();
    s.metricsText =
        capturing ? s.rateText + QStringLiteral(" · ") + s.latencyText : tr("Not capturing");
    s.visibleCount = static_cast<int>(camera.visibleTagIds.size());
    s.seesTags = s.visibleCount > 0;
    s.tagsText = tagsText(camera.visibleTagIds);
    s.requestText = requestText(camera.vendor, robot);
    return s;
}

const sdk::CameraRobotState* matchRobotState(const sdk::CameraSample& camera,
                                             const std::string& networkName,
                                             std::span<const sdk::CameraRobotState> detected) {
    for (const sdk::CameraRobotState& robot : detected)
        if (!robot.cameraId.empty() && robot.cameraId == camera.id) return &robot;
    if (!networkName.empty())
        for (const sdk::CameraRobotState& robot : detected)
            if (robot.cameraId.empty() && robot.vendor == camera.vendor &&
                robot.ntName == networkName)
                return &robot;
    return nullptr;
}

// ---- notices
// ----------------------------------------------------------------------------------------
QString visionNoticeTitle(const std::string& code) {
    namespace vc = sdk::validation_code;
    if (code == vc::kVisionLayoutMissing) return tr("AprilTag layout not found");
    if (code == vc::kVisionCameraUnconfigured) return tr("Camera not configured");
    if (code == vc::kVisionCameraStatus) return tr("Camera problem");
    return isVisionCode(code) ? tr("Vision warning") : QString();
}

std::vector<VisionNotice> visionNotices(const sdk::ISimSnapshot* snapshot) {
    std::vector<VisionNotice> out;
    if (snapshot == nullptr) return out;
    for (const sdk::HubWarning& warning : snapshot->warnings()) {
        if (!isVisionCode(warning.code)) continue;
        VisionNotice notice;
        notice.code = QString::fromStdString(warning.code);
        notice.title = visionNoticeTitle(warning.code);
        notice.detail = warning.message.empty() ? defaultDetail(warning.code)
                                                : QString::fromStdString(warning.message);
        notice.severity = warning.severity;
        out.push_back(std::move(notice));
    }
    return out;
}

VisionNoticeModel::VisionNoticeModel(QObject* parent) : QAbstractListModel(parent) {}

void VisionNoticeModel::update(const sdk::ISimSnapshot* snapshot) {
    if (notices_.empty()) {  // the common case: nothing now and nothing before
        const bool any = snapshot != nullptr &&
                         std::any_of(snapshot->warnings().begin(), snapshot->warnings().end(),
                                     [](const sdk::HubWarning& w) { return isVisionCode(w.code); });
        if (!any) return;
    }
    std::vector<VisionNotice> next = visionNotices(snapshot);
    if (next == notices_) return;
    std::vector<VisionNotice> raised;
    for (const VisionNotice& notice : next)
        if (std::find(notices_.begin(), notices_.end(), notice) == notices_.end())
            raised.push_back(notice);
    beginResetModel();
    notices_ = std::move(next);
    endResetModel();
    for (const VisionNotice& notice : raised)
        emit noticeRaised(notice.title, notice.detail, static_cast<int>(notice.severity));
}

int VisionNoticeModel::rowCount(const QModelIndex& parent) const {
    return parent.isValid() ? 0 : static_cast<int>(notices_.size());
}

QVariant VisionNoticeModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= rowCount()) return {};
    const VisionNotice& n = notices_[static_cast<std::size_t>(index.row())];
    switch (role) {
        case TitleRole:
            return n.title;
        case DetailRole:
            return n.detail;
        case SeverityRole:
            return static_cast<int>(n.severity);
        case ToneRole:
            return toneColor(n.severity == sdk::Severity::Error     ? Tone::Error
                             : n.severity == sdk::Severity::Warning ? Tone::Warning
                                                                    : Tone::Info);
        default:
            return {};
    }
}

QHash<int, QByteArray> VisionNoticeModel::roleNames() const {
    return {{TitleRole, "title"},
            {DetailRole, "detail"},
            {SeverityRole, "severity"},
            {ToneRole, "tone"}};
}

// ---- the model of cards
// -----------------------------------------------------------------------------
CameraHudModel::CameraHudModel(QObject* parent) : QAbstractListModel(parent), notices_(this) {}

void CameraHudModel::update(const sdk::ISimSnapshot* snapshot) {
    notices_.update(snapshot);
    const std::span<const sdk::CameraSample> cameras =
        snapshot != nullptr ? snapshot->cameras() : std::span<const sdk::CameraSample>{};
    const sdk::SimTimeUs now = snapshot != nullptr ? snapshot->simTimeUs() : 0;

    bool structural = cameras.size() != cards_.size();
    for (std::size_t i = 0; !structural && i < cameras.size(); ++i)
        structural = ids_[i] != cameras[i].id;
    if (structural) meters_.assign(cameras.size(), RateMeter{});

    const auto evaluate = [&](std::size_t i) {
        meters_[i].add(cameras[i].framesCaptured, now);
        std::string networkName;
        if (config_)
            if (const sdk::CameraConfig* configured = config_->vision.findCamera(cameras[i].id))
                networkName = configured->networkName();
        const sdk::CameraRobotState* robot =
            matchRobotState(cameras[i], networkName, snapshot->detectedCameras());
        return evaluateCameraCard(cameras[i], robot, meters_[i].hz());
    };

    if (structural) {
        std::vector<CameraCardState> next;
        next.reserve(cameras.size());
        for (std::size_t i = 0; i < cameras.size(); ++i) next.push_back(evaluate(i));
        beginResetModel();
        cards_ = std::move(next);
        ids_.clear();
        for (const sdk::CameraSample& camera : cameras) ids_.push_back(camera.id);
        endResetModel();
        return;
    }
    for (std::size_t i = 0; i < cameras.size(); ++i) {
        CameraCardState next = evaluate(i);
        if (next == cards_[i]) continue;
        cards_[i] = std::move(next);
        const QModelIndex idx = index(static_cast<int>(i));
        emit dataChanged(idx, idx);
    }
}

int CameraHudModel::rowCount(const QModelIndex& parent) const {
    return parent.isValid() ? 0 : static_cast<int>(cards_.size());
}

QVariant CameraHudModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= rowCount()) return {};
    const CameraCardState& c = cards_[static_cast<std::size_t>(index.row())];
    switch (role) {
        case IdRole:
            return c.id;
        case LabelRole:
            return c.label;
        case VendorRole:
            return c.vendor;
        case StateRole:
            return static_cast<int>(c.state);
        case StateTextRole:
            return c.stateText;
        case ToneRole:
            return toneColor(toneOf(c.state));
        case RateRole:
            return c.rateText;
        case LatencyRole:
            return c.latencyText;
        case MetricsRole:
            return c.metricsText;
        case TagsRole:
            return c.tagsText;
        case RequestRole:
            return c.requestText;
        case ActiveRole:
            return c.seesTags;
        default:
            return {};
    }
}

QHash<int, QByteArray> CameraHudModel::roleNames() const {
    return {{IdRole, "cameraId"},     {LabelRole, "label"},         {VendorRole, "vendor"},
            {StateRole, "stateKind"}, {StateTextRole, "stateText"}, {ToneRole, "tone"},
            {RateRole, "rate"},       {LatencyRole, "latency"},     {MetricsRole, "metrics"},
            {TagsRole, "tags"},       {RequestRole, "request"},     {ActiveRole, "active"}};
}

}  // namespace wpiemu::app::runview
