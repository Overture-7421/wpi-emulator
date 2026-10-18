#include "wpiemu/bridges/vision/vision_bridge.h"

#include <algorithm>
#include <exception>
#include <string>
#include <utility>

#include "wpiemu/bridges/vision/limelight_vendor.h"
#include "wpiemu/bridges/vision/nt_binding.h"
#include "wpiemu/bridges/vision/null_nt_api.h"
#include "wpiemu/bridges/vision/photon_vendor.h"

namespace wpiemu::bridges::vision {

namespace {

constexpr const char* kCategory = "bridge.vision";
// Distinct unknown camera ids that get their own warning: a stream of garbage ids must not grow memory.
constexpr std::size_t kMaxUnknownCameraWarnings = 32;
// Unconfigured cameras the bridge keeps track of and reports. A robot that publishes request topics under endless names
// must not grow every ActuatorFrame (the IPC message caps are not visible to this module).
constexpr std::size_t kMaxDetectedCameras = 64;
constexpr std::size_t kNoCamera = static_cast<std::size_t>(-1);

const char* vendorName(sdk::CameraVendor kind) {
    switch (kind) {
        case sdk::CameraVendor::PhotonVision:
            return "PhotonVision";
        case sdk::CameraVendor::Limelight:
            return "Limelight";
    }
    return "unknown";
}

// CameraRobotState equality that ignores framesPublished, the one field that changes with every frame. `current` is
// patched in place and restored (no copy: this runs for every camera on every tick).
bool sameExceptFrames(sdk::CameraRobotState& current, const sdk::CameraRobotState& last) {
    const std::uint64_t frames = current.framesPublished;
    current.framesPublished = last.framesPublished;
    const bool same = current == last;
    current.framesPublished = frames;
    return same;
}

std::string quoted(const std::string& text) {
    return "'" + text + "'";
}

}  // namespace

// Runs one call into a publisher or vendor. An exception disables that camera alone (its publisher is never called
// again) and is logged once; nothing propagates.
template <class Fn>
bool VisionBridge::guarded(Camera& camera, const char* what, Fn&& fn) noexcept {
    try {
        fn();
        return true;
    } catch (const std::exception& e) {
        disableCamera(camera, what, e.what());
    } catch (...) {
        disableCamera(camera, what, "unknown exception");
    }
    return false;
}

VisionBridge::VisionBridge(VisionBridgeOptions options) : options_(std::move(options)) {}

VisionBridge::~VisionBridge() {
    stop();
}

// ---- lifecycle ---------------------------------------------------------------------------------------------------

sdk::Status VisionBridge::start(sdk::IBridgeHost& host) {
    if (stopped_) return sdk::makeError(sdk::ErrorCode::InvalidState, "the vision bridge was stopped");
    try {
        log_ = &host.logger();
        if (!nt_) {
            nt_ = options_.nt ? options_.nt : std::shared_ptr<INtApi>(makeNtCoreApi());
            if (!nt_) nt_ = std::make_shared<NullNtApi>("no ntcore binding");
        }
        if (!vendorsBuilt_) buildVendors();
    } catch (const std::exception& e) {
        log_ = nullptr;
        return sdk::makeError(sdk::ErrorCode::Internal, std::string("the vision bridge could not start: ") + e.what());
    } catch (...) {
        log_ = nullptr;
        return sdk::makeError(sdk::ErrorCode::Internal, "the vision bridge could not start");
    }
    host_ = &host;  // from here on the bridge is live, but it touches ntcore only from poll()
    return sdk::okStatus();
}

void VisionBridge::stop() noexcept {
    // Nothing else: publishers and NT handles are never released or unpublished (ntcore may be gone), they are only
    // destroyed with the bridge. After this no method reaches the host, the logger, INtApi or a publisher.
    stopped_ = true;
    host_ = nullptr;
    log_ = nullptr;
}

void VisionBridge::buildVendors() {
    std::vector<VendorSlot> slots;
    const auto add = [&](std::shared_ptr<IVisionVendor> vendor) {
        if (!vendor) return;
        try {
            const sdk::CameraVendor kind = vendor->vendor();
            slots.push_back(VendorSlot{std::move(vendor), kind});
        } catch (...) {
            ++stats_.failures;
            if (log_ != nullptr) log_->error(kCategory, "a vision vendor threw while it was registered: ignored");
        }
    };
    if (options_.vendors) {
        for (const std::shared_ptr<IVisionVendor>& vendor : *options_.vendors) add(vendor);
    } else {
        add(std::shared_ptr<IVisionVendor>(makePhotonVendor()));
        add(std::shared_ptr<IVisionVendor>(makeLimelightVendor()));
    }
    vendors_ = std::move(slots);
    vendorsBuilt_ = true;
    stats_.vendors = vendors_.size();
}

IVisionVendor* VisionBridge::vendorFor(sdk::CameraVendor kind) const {
    for (const VendorSlot& slot : vendors_)
        if (slot.kind == kind) return slot.vendor.get();
    return nullptr;
}

void VisionBridge::poll() {
    if (!running()) return;
    try {
        pollImpl();
    } catch (...) {
        internalError("poll");
    }
}

void VisionBridge::pollImpl() {
    syncSetup();
    const bool bound = nt_->available();
    if (bound != ntBound_) onNtBoundChanged(bound);
    if (!programStarted_ && host_->programStarted()) programStarted_ = true;
    if (!bound || !programStarted_) {
        if (programStarted_) warnNtUnavailable();
        return;
    }

    startPendingCameras();
    pollCameras();
    if (vendors_.empty()) return;  // nobody to ask: nothing could be discovered
    const std::int64_t now = nt_->nowUs();
    if (discoveryDue(now)) runDiscovery(now);
}

void VisionBridge::onNtBoundChanged(bool bound) {
    ntBound_ = bound;
    if (bound && warnedNtUnavailable_) log_->info(kCategory, "ntcore is available now: the virtual cameras can start");
    for (Camera& camera : cameras_)
        if (camera.phase == Phase::Waiting) refreshPlaceholder(camera);
}

void VisionBridge::warnNtUnavailable() {
    if (warnedNtUnavailable_) return;
    warnedNtUnavailable_ = true;
    const std::string_view reason = nt_->unavailableReason();
    std::string message = "NetworkTables (ntcore) is not available in the robot process";
    if (!reason.empty()) message += ": " + std::string(reason);
    message += "; the virtual cameras stay off until it is";
    log_->warn(kCategory, message);
}

// ---- camera setup ------------------------------------------------------------------------------------------------

void VisionBridge::syncSetup() {
    const std::uint64_t revision = host_->visionSetupRevision();
    if (setupAdopted_ && revision == setupRevision_) return;
    adoptSetup(host_->visionSetup());
    setupAdopted_ = true;
    setupRevision_ = revision;
}

// Builds the camera list of a (new) HelloAck. Cameras whose CameraSetup is unchanged keep their publisher and report
// state; new and changed ones start over (a changed camera's old publisher goes quiet: its topics stay, nothing is
// unpublished) and cameras that disappeared go quiet. All that can throw happens before anything moves.
void VisionBridge::adoptSetup(const sdk::VisionSetup& setup) {
    std::vector<Camera> next;
    std::vector<std::size_t> reuse;  // per entry of `next`: the index in cameras_ whose publisher carries over
    next.reserve(setup.cameras.size());
    reuse.reserve(setup.cameras.size());
    std::size_t created = 0;
    std::size_t replaced = 0;
    for (const sdk::CameraSetup& wanted : setup.cameras) {
        const auto duplicate = [&](const Camera& camera) { return camera.setup.cameraId == wanted.cameraId; };
        if (std::any_of(next.begin(), next.end(), duplicate)) {
            if (log_ != nullptr)
                log_->warn(kCategory, "the camera setup lists camera " + quoted(wanted.cameraId) +
                                          " twice: the second entry is ignored");
            continue;
        }
        const auto previous = std::find_if(cameras_.begin(), cameras_.end(), duplicate);
        next.emplace_back();
        Camera& camera = next.back();
        camera.setup = wanted;
        if (previous != cameras_.end() && previous->setup == wanted) {
            reuse.push_back(static_cast<std::size_t>(previous - cameras_.begin()));
            continue;
        }
        reuse.push_back(kNoCamera);
        camera.vendor = vendorFor(wanted.vendor);
        camera.phase = camera.vendor != nullptr ? Phase::Waiting : Phase::NoVendor;
        refreshPlaceholder(camera);
        if (previous != cameras_.end()) {
            ++replaced;
        } else {
            ++created;
        }
    }

    // Commit: nothing below throws.
    const std::size_t kept = static_cast<std::size_t>(
        std::count_if(reuse.begin(), reuse.end(), [](std::size_t i) { return i != kNoCamera; }));
    const std::size_t removed = cameras_.size() - kept - replaced;
    for (std::size_t i = 0; i < next.size(); ++i)
        if (reuse[i] != kNoCamera) next[i] = std::move(cameras_[reuse[i]]);
    cameras_ = std::move(next);  // the replaced and removed publishers are destroyed here, without touching NT
    warnedUnknownCameras_.clear();

    if (log_ == nullptr) return;
    if (created + replaced + removed > 0)
        log_->info(kCategory, "camera setup: " + std::to_string(cameras_.size()) + " camera(s), " +
                                  std::to_string(kept) + " kept, " + std::to_string(created) + " new, " +
                                  std::to_string(replaced) + " changed, " + std::to_string(removed) + " removed");
    for (Camera& camera : cameras_)
        if (camera.phase == Phase::NoVendor) markNoVendor(camera);
}

void VisionBridge::startPendingCameras() {
    for (Camera& camera : cameras_) {
        if (camera.phase != Phase::Waiting) continue;
        if (camera.vendor == nullptr) {
            markNoVendor(camera);
            continue;
        }
        std::unique_ptr<ICameraPublisher> publisher;
        if (!guarded(camera, "createPublisher",
                     [&] { publisher = camera.vendor->createPublisher(*nt_, *log_, camera.setup); }))
            continue;
        if (!publisher) {
            markNoVendor(camera);
            continue;
        }
        camera.publisher = std::move(publisher);
        if (!guarded(camera, "start", [&] { camera.publisher->start(); })) continue;
        setPhase(camera, Phase::Running);
        ++stats_.publishersStarted;
        log_->info(kCategory, std::string("camera ") + quoted(camera.setup.cameraId) + " (" +
                                  vendorName(camera.setup.vendor) + " " + quoted(camera.setup.ntName) +
                                  "): virtual coprocessor started");
    }
}

void VisionBridge::pollCameras() {
    for (Camera& camera : cameras_)
        if (camera.phase == Phase::Running) guarded(camera, "poll", [&] { camera.publisher->poll(); });
}

void VisionBridge::markNoVendor(Camera& camera) {
    setPhase(camera, Phase::NoVendor);
    if (camera.loggedNoVendor || log_ == nullptr) return;
    camera.loggedNoVendor = true;
    log_->warn(kCategory, "camera " + quoted(camera.setup.cameraId) + ": this build has no " +
                              vendorName(camera.setup.vendor) + " support (vendor_unavailable); the camera stays off");
}

void VisionBridge::setPhase(Camera& camera, Phase phase) {
    camera.phase = phase;
    refreshPlaceholder(camera);
}

// What a configured camera reports while it has no running publisher. Built on every state change, never per tick.
void VisionBridge::refreshPlaceholder(Camera& camera) const {
    std::string_view status = camera_status::kOk;
    switch (camera.phase) {
        case Phase::Waiting:
            status = ntBound_ ? camera_status::kOk : camera_status::kNtcoreUnavailable;
            break;
        case Phase::Running:
            break;
        case Phase::NoVendor:
            status = camera_status::kVendorUnavailable;
            break;
        case Phase::Disabled:
            status = camera_status::kDisabled;
            break;
    }
    sdk::CameraRobotState& state = camera.placeholder;
    state = sdk::CameraRobotState{};
    state.ntName = camera.setup.ntName;
    state.vendor = camera.setup.vendor;
    state.cameraId = camera.setup.cameraId;
    state.present = false;
    state.status.assign(status);
}

void VisionBridge::disableCamera(Camera& camera, const char* what, const char* reason) noexcept {
    ++stats_.failures;
    camera.phase = Phase::Disabled;
    try {
        refreshPlaceholder(camera);
        if (camera.loggedFailure || log_ == nullptr) return;
        camera.loggedFailure = true;
        log_->error(kCategory, "camera " + quoted(camera.setup.cameraId) + ": " + what + " threw (" + reason +
                                   "); the camera is disabled");
    } catch (...) {
    }
}

void VisionBridge::internalError(const char* where) noexcept {
    ++stats_.failures;
    if (loggedInternalError_ || log_ == nullptr) return;
    loggedInternalError_ = true;
    try {
        log_->error(kCategory,
                    std::string(where) + " failed unexpectedly (out of memory?); camera reports may be out of date");
    } catch (...) {
    }
}

// ---- frames ------------------------------------------------------------------------------------------------------

void VisionBridge::applyVision(std::span<const sdk::VisionFrame> frames) {
    if (frames.empty()) return;
    if (!running()) {
        stats_.framesDropped += frames.size();
        return;
    }
    try {
        for (const sdk::VisionFrame& frame : frames) routeFrame(frame);
    } catch (...) {
        internalError("applyVision");
    }
}

void VisionBridge::routeFrame(const sdk::VisionFrame& frame) {
    Camera* target = nullptr;
    for (Camera& camera : cameras_) {
        if (camera.setup.cameraId == frame.cameraId) {
            target = &camera;
            break;
        }
    }
    if (target == nullptr) {
        ++stats_.framesDropped;
        warnUnknownCamera(frame.cameraId);
        return;
    }
    if (target->phase == Phase::Running && guarded(*target, "publish", [&] { target->publisher->publish(frame); })) {
        ++stats_.framesRouted;
        return;
    }
    ++stats_.framesDropped;  // the camera is waiting, unsupported or disabled: its state says why
}

void VisionBridge::warnUnknownCamera(const std::string& cameraId) {
    if (log_ == nullptr || warnedUnknownCameras_.size() >= kMaxUnknownCameraWarnings) return;
    if (std::find(warnedUnknownCameras_.begin(), warnedUnknownCameras_.end(), cameraId) != warnedUnknownCameras_.end())
        return;
    warnedUnknownCameras_.push_back(cameraId);
    log_->warn(kCategory,
               "dropping frames of camera " + quoted(cameraId) + ": it is not in the camera setup of this connection");
}

// ---- discovery ---------------------------------------------------------------------------------------------------

bool VisionBridge::discoveryDue(std::int64_t nowUs) const {
    return !discoveryRan_ || nowUs < lastDiscoveryUs_ || nowUs - lastDiscoveryUs_ >= options_.discoveryIntervalUs;
}

// Lists the topics below each vendor's prefix and lets the vendor name the cameras the ROBOT uses. Cameras that match
// no configured camera are remembered (and reported) as unconfigured detections.
void VisionBridge::runDiscovery(std::int64_t nowUs) {
    discoveryRan_ = true;
    lastDiscoveryUs_ = nowUs;
    ++stats_.discoveryRuns;
    for (VendorSlot& slot : vendors_) {
        if (slot.discoveryFailed) continue;
        topicScratch_.clear();
        nameScratch_.clear();
        try {
            nt_->topics(slot.vendor->topicPrefix(), topicScratch_);
            slot.vendor->discoverRobotCameras(topicScratch_, nameScratch_);
        } catch (const std::exception& e) {
            slot.discoveryFailed = true;
            ++stats_.failures;
            log_->error(kCategory, std::string("camera discovery of ") + vendorName(slot.kind) + " threw (" + e.what() +
                                       "): it is off");
            continue;
        } catch (...) {
            slot.discoveryFailed = true;
            ++stats_.failures;
            log_->error(kCategory, std::string("camera discovery of ") + vendorName(slot.kind) + " threw: it is off");
            continue;
        }
        updateDetected(slot.kind, nameScratch_);
    }
}

bool VisionBridge::isConfigured(sdk::CameraVendor kind, const std::string& ntName) const {
    return std::any_of(cameras_.begin(), cameras_.end(), [&](const Camera& camera) {
        return camera.setup.vendor == kind && camera.setup.ntName == ntName;
    });
}

void VisionBridge::updateDetected(sdk::CameraVendor kind, const std::vector<std::string>& names) {
    for (Detected& detected : detected_)
        if (detected.state.vendor == kind) detected.seen = false;
    for (const std::string& name : names) {
        if (name.empty() || isConfigured(kind, name)) continue;  // a configured camera reports itself
        const auto known = std::find_if(detected_.begin(), detected_.end(), [&](const Detected& detected) {
            return detected.state.vendor == kind && detected.state.ntName == name;
        });
        if (known == detected_.end()) {
            if (detected_.size() >= kMaxDetectedCameras) {
                if (!warnedTooManyDetected_) {
                    warnedTooManyDetected_ = true;
                    log_->warn(kCategory, "the robot uses more than " + std::to_string(kMaxDetectedCameras) +
                                              " unconfigured cameras: the rest are ignored");
                }
                continue;
            }
            Detected detected;
            detected.state.ntName = name;
            detected.state.vendor = kind;
            detected.state.present = true;
            detected.seen = true;
            detected_.push_back(std::move(detected));
            log_->info(kCategory, std::string("the robot uses an unconfigured ") + vendorName(kind) + " camera " +
                                      quoted(name) + ": add it to the project to simulate it");
            continue;
        }
        known->seen = true;
        if (!known->state.present) {
            known->state.present = true;
            known->reportPending = true;
        }
    }
    for (Detected& detected : detected_) {
        if (detected.state.vendor != kind || detected.seen || !detected.state.present) continue;
        detected.state.present = false;  // the robot stopped using it
        detected.reportPending = true;
    }
}

// ---- reports -----------------------------------------------------------------------------------------------------

void VisionBridge::collectVision(std::vector<sdk::CameraRobotState>& out, bool full) {
    if (!running()) return;
    try {
        collectImpl(out, full);
    } catch (...) {
        internalError("collectVision");
    }
}

bool VisionBridge::framesReportDue(std::int64_t nowUs, std::int64_t lastReportUs) const {
    return nowUs < lastReportUs || nowUs - lastReportUs >= options_.framesReportIntervalUs;
}

void VisionBridge::collectImpl(std::vector<sdk::CameraRobotState>& out, bool full) {
    std::optional<std::int64_t> now;  // read lazily: the common tick has nothing to report
    const auto nowUs = [&] {
        if (!now) now = nt_->nowUs();
        return *now;
    };

    sdk::CameraRobotState fresh;
    for (Camera& camera : cameras_) {
        sdk::CameraRobotState* state = &camera.placeholder;
        if (camera.phase == Phase::Running &&
            guarded(camera, "robotState", [&] { fresh = camera.publisher->robotState(); })) {
            if (fresh.cameraId != camera.setup.cameraId) fresh.cameraId = camera.setup.cameraId;
            if (fresh.ntName.empty()) fresh.ntName = camera.setup.ntName;
            fresh.vendor = camera.setup.vendor;
            state = &fresh;
        }

        bool report = full || !camera.hasReported;
        if (!report) {
            if (!sameExceptFrames(*state, camera.lastReported)) {
                report = true;  // any change but the frame counter is reported at once
            } else if (state->framesPublished != camera.lastReported.framesPublished) {
                report = framesReportDue(nowUs(), camera.lastReportUs);
            }
        }
        if (!report) continue;
        out.push_back(*state);
        camera.lastReported = *state;
        camera.hasReported = true;
        camera.lastReportUs = nowUs();
    }

    for (Detected& detected : detected_) {
        if (full) {
            if (detected.state.present) out.push_back(detected.state);
            detected.reportPending = false;
        } else if (detected.reportPending) {
            out.push_back(detected.state);
            detected.reportPending = false;
        }
    }
}

}  // namespace wpiemu::bridges::vision
