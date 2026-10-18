// Manual-test fake emulator for halsim_wpiemu (NOT run by ctest). It plays the emulator side of the
// protocol over the real local transport: accepts the extension, answers Hello, waits for the robot
// program to start, enables the robot, then single-steps lock-step ticks and prints what the robot
// created (devices) and did (outputs). See halsim_ext/tests/manual/README.md for how to point a
// real WPILib project at it.
//
//   wpiemu_halsim_fake_emulator [--endpoint-file F] [--seconds S] [--step-us N] [--no-enable]
//                               [--accept-timeout-s N] [--start-timeout-s N] [--dump-outputs]
//                               [--vision]
//
// --vision adds two synthetic cameras (a PhotonVision "photon-front" and a Limelight
// "limelight-rear", vision_scenario.h): their setup rides the HelloAck, camera frames ride the
// SensorFrames and the camera states the robot process sends back are printed (`CAMERA ...` lines).
//
// Prints `WPIEMU_IPC_ENDPOINT=<address>` (and writes it to --endpoint-file) before it blocks in
// accept().
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <variant>

#include "vision_scenario.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/ipc/endpoint.h"
#include "wpiemu/ipc/handshake.h"
#include "wpiemu/ipc/local_ipc.h"
#include "wpiemu/sdk/ipc/lifecycle.h"
#include "wpiemu/sdk/ipc/lockstep.h"

using namespace wpiemu;
using namespace std::chrono_literals;
namespace proto = sdk::proto;

namespace {

struct Options {
    std::string endpointFile;
    double seconds = 5.0;
    std::uint32_t stepUs = 5000;
    bool enable = true;
    int acceptTimeoutS = 600;
    int startTimeoutS = 180;
    bool dumpOutputs = false;
    bool vision = false;
};

bool parse(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
        if (a == "--endpoint-file") {
            if (const char* v = next()) o.endpointFile = v;
            else return false;
        } else if (a == "--seconds") {
            if (const char* v = next()) o.seconds = std::atof(v);
            else return false;
        } else if (a == "--step-us") {
            if (const char* v = next()) o.stepUs = static_cast<std::uint32_t>(std::atoi(v));
            else return false;
        } else if (a == "--accept-timeout-s") {
            if (const char* v = next()) o.acceptTimeoutS = std::atoi(v);
            else return false;
        } else if (a == "--start-timeout-s") {
            if (const char* v = next()) o.startTimeoutS = std::atoi(v);
            else return false;
        } else if (a == "--no-enable") {
            o.enable = false;
        } else if (a == "--dump-outputs") {
            o.dumpOutputs = true;
        } else if (a == "--vision") {
            o.vision = true;
        } else {
            return false;
        }
    }
    return o.stepUs > 0 && o.seconds > 0;
}

// performServerHandshake cannot attach a camera setup to the HelloAck, so --vision answers the
// Hello itself (what that function does, plus HelloAck::vision).
sdk::Result<ipc::ServerHandshakeResult> handshakeWithVision(
    proto::MessageChannel& channel, const ipc::ServerHandshakeConfig& config,
    const sdk::VisionSetup& vision, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    proto::Message first;
    for (;;) {
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        if (left.count() <= 0) return sdk::makeError(sdk::ErrorCode::Timeout, "no Hello received");
        const sdk::Result<bool> got = channel.receive(first, std::min(left, 200ms));
        if (!got) {
            if (got.error().code == sdk::ErrorCode::Unsupported) continue;  // unknown message type
            return got.error();
        }
        if (got.value()) break;
    }
    const proto::Hello* hello = std::get_if<proto::Hello>(&first);
    if (hello == nullptr)
        return sdk::makeError(sdk::ErrorCode::ProtocolError, "the first message must be Hello");

    proto::HelloAck ack;
    ack.sessionId = config.sessionId;
    ack.runEpoch = config.runEpoch;
    ack.clockMode = config.clockMode;
    ack.stepUs = config.stepUs;
    ack.discoveryOnly = config.discoveryOnly;
    ack.enableL3Hooks = config.enableL3Hooks;
    ack.watchdogPolicy = config.watchdogPolicy;
    ack.watchdogTimeoutMs =
        proto::negotiateWatchdogTimeoutMs(hello->proposedWatchdogMs, config.preferredWatchdogMs);
    ack.vision = vision;
    if (sdk::Status sent = channel.send(proto::Message(ack)); !sent) return sent.error();
    return ipc::ServerHandshakeResult{*hello, std::move(ack)};
}

const char* vendorName(sdk::CameraVendor vendor) {
    return vendor == sdk::CameraVendor::Limelight ? "limelight" : "photonvision";
}

// One line per camera state the robot process reported (informational ActuatorFrame::vision).
void printCameraState(std::uint64_t tick, const sdk::CameraRobotState& s) {
    std::printf(
        "CAMERA tick=%llu name='%s' vendor=%s id='%s' present=%d publishing=%d status='%s' "
        "frames=%llu",
        static_cast<unsigned long long>(tick), s.ntName.c_str(), vendorName(s.vendor),
        s.cameraId.c_str(), s.present ? 1 : 0, s.publishing ? 1 : 0, s.status.c_str(),
        static_cast<unsigned long long>(s.framesPublished));
    if (s.pipelineIndex) std::printf(" pipeline=%d", *s.pipelineIndex);
    if (s.ledMode) std::printf(" led=%d", *s.ledMode);
    if (s.imuMode) std::printf(" imu=%d", *s.imuMode);
    if (s.driverMode) std::printf(" driver=%d", *s.driverMode ? 1 : 0);
    if (s.throttle) std::printf(" throttle=%g", *s.throttle);
    if (s.fpsLimit) std::printf(" fpsLimit=%g", *s.fpsLimit);
    const auto array = [](const char* label, const std::optional<std::array<double, 6>>& values) {
        if (!values) return;
        std::printf(" %s=[%g,%g,%g,%g,%g,%g]", label, (*values)[0], (*values)[1], (*values)[2],
                    (*values)[3], (*values)[4], (*values)[5]);
    };
    array("orientation", s.robotOrientationDeg);
    array("poseOverride", s.cameraPoseOverride);
    std::printf("\n");
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parse(argc, argv, opt)) {
        std::fprintf(stderr,
                     "usage: %s [--endpoint-file F] [--seconds S] [--step-us N] [--no-enable] "
                     "[--dump-outputs] [--vision]\n",
                     argv[0]);
        return 64;
    }
    std::optional<halsim::manual::VisionScenario> scenario;
    if (opt.vision) scenario.emplace(opt.stepUs);

    const ipc::ConnectionEndpoint endpoint = ipc::ConnectionEndpoint::generate();
    auto server = ipc::makeLocalServer(endpoint);
    if (!server || !server.value()->listen()) {
        std::fprintf(stderr, "cannot listen on %s\n", endpoint.address().c_str());
        return 1;
    }
    std::printf("WPIEMU_IPC_ENDPOINT=%s\n", endpoint.address().c_str());
    std::fflush(stdout);
    if (!opt.endpointFile.empty()) {
        std::ofstream f(opt.endpointFile, std::ios::trunc);
        f << endpoint.address();
    }

    auto accepted = server.value()->accept(std::chrono::seconds(opt.acceptTimeoutS));
    if (!accepted || !accepted.value()) {
        std::fprintf(stderr, "no robot connected within %d s\n", opt.acceptTimeoutS);
        return 2;
    }
    proto::MessageChannel channel(std::move(accepted).value(), std::make_shared<ipc::BinaryMessageCodec>());

    ipc::ServerHandshakeConfig hs;
    hs.stepUs = opt.stepUs;
    hs.sessionId = 1;
    hs.runEpoch = proto::kFirstRunEpoch;
    auto hello = scenario ? handshakeWithVision(channel, hs, scenario->setup(), 30s)
                          : ipc::performServerHandshake(channel, hs, 30s);
    if (!hello) {
        std::fprintf(stderr, "handshake failed: %s\n", hello.error().message.c_str());
        return 3;
    }
    const proto::Hello& h = hello.value().hello;
    std::printf("HELLO pid=%lld lang=%d wpilib=%s program=%s watchdog=%ums\n", static_cast<long long>(h.pid),
                static_cast<int>(h.language), h.wpilibVersion.c_str(), h.programName.c_str(),
                hello.value().ack.watchdogTimeoutMs);
    for (const sdk::CameraSetup& c : hello.value().ack.vision.cameras)
        std::printf("VISION_SETUP camera='%s' vendor=%s size=%dx%d rate=%gHz latency=%lldus\n",
                    c.cameraId.c_str(), vendorName(c.vendor), c.widthPx, c.heightPx, c.frameRateHz,
                    static_cast<long long>(c.latencyUs));

    std::map<std::uint32_t, proto::DeviceAnnounce> devices;
    std::map<std::uint64_t, double> lastOutput;  // (device << 16 | signal) -> value
    std::uint64_t cameraStates = 0;              // CameraRobotState entries received (--vision)
    proto::LockstepWindow window;
    window.begin(hs.sessionId, hs.runEpoch);

    auto handle = [&](const proto::Message& m, proto::ActuatorFrame* ackOut) {
        if (const auto* a = std::get_if<proto::DeviceAnnounce>(&m)) {
            const bool fresh = devices.find(a->id.value) == devices.end();
            devices[a->id.value] = *a;
            if (fresh)
                std::printf("DEVICE #%u %s/%s id=%d ch=%d bus='%s' native='%s' signals=%zu%s\n", a->id.value,
                            a->descriptor.family.c_str(), a->descriptor.type.c_str(), a->descriptor.id,
                            a->descriptor.channel, a->descriptor.bus.c_str(), a->descriptor.nativeName.c_str(),
                            a->signalSpecs.size(), a->retired ? " (retired)" : "");
        } else if (const auto* l = std::get_if<proto::Log>(&m)) {
            if (!proto::isLifecycleLog(*l, proto::kLifecycleProgramStarted))
                std::printf("LOG [%s] %s\n", l->source.c_str(), l->message.c_str());
        } else if (const auto* f = std::get_if<proto::ActuatorFrame>(&m)) {
            if (ackOut != nullptr) *ackOut = *f;
            for (const sdk::CameraRobotState& s : f->vision) {
                ++cameraStates;
                printCameraState(f->tick, s);
            }
            if (opt.dumpOutputs) {
                for (const auto& s : f->samples) {
                    const std::uint64_t key = (static_cast<std::uint64_t>(s.device.value) << 16) | s.signal;
                    const auto it = devices.find(s.device.value);
                    const char* sig = it != devices.end() && s.signal < it->second.signalSpecs.size()
                                          ? it->second.signalSpecs[s.signal].name.c_str()
                                          : "?";
                    if (lastOutput.find(key) == lastOutput.end() || lastOutput[key] != s.value)
                        std::printf("OUT tick=%llu dev=#%u %s = %g\n", static_cast<unsigned long long>(f->tick),
                                    s.device.value, sig, s.value);
                    lastOutput[key] = s.value;
                }
            }
        }
    };

    // 1) wait until the robot constructor finished (every device it created is announced before this).
    bool started = false;
    const auto startDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(opt.startTimeoutS);
    while (!started && std::chrono::steady_clock::now() < startDeadline) {
        proto::Message m;
        const auto got = channel.receive(m, 100ms);
        if (!got) {
            std::fprintf(stderr, "connection lost while waiting for the program: %s\n", got.error().message.c_str());
            return 4;
        }
        if (!got.value()) continue;
        if (const auto* l = std::get_if<proto::Log>(&m);
            l != nullptr && proto::isLifecycleLog(*l, proto::kLifecycleProgramStarted))
            started = true;
        handle(m, nullptr);
    }
    if (!started) {
        std::fprintf(stderr, "robot program did not start within %d s\n", opt.startTimeoutS);
        return 5;
    }
    std::printf("PROGRAM_STARTED devices=%zu\n", devices.size());

    // 2) enable the robot and step.
    if (opt.enable) {
        proto::DsState ds;
        ds.enabled = true;
        ds.sessionId = hs.sessionId;
        ds.runEpoch = hs.runEpoch;
        (void)channel.send(proto::Message(ds));
    }
    const std::uint64_t ticks = static_cast<std::uint64_t>(opt.seconds * 1e6 / opt.stepUs);
    std::uint64_t completed = 0;
    std::uint64_t lastLoops = 0;
    std::uint64_t visionFramesSent = 0;
    const auto t0 = std::chrono::steady_clock::now();
    for (std::uint64_t t = 1; t <= ticks; ++t) {
        proto::SensorFrame f;
        f.tick = t;
        f.simTimeUs = static_cast<sdk::SimTimeUs>(t * opt.stepUs);
        f.stepUs = opt.stepUs;
        f.sessionId = hs.sessionId;
        f.runEpoch = hs.runEpoch;
        if (scenario) f.vision = scenario->framesFor(t);
        visionFramesSent += f.vision.size();
        if (!window.onSent(f) || !channel.send(proto::Message(f))) {
            std::fprintf(stderr, "send failed at tick %llu\n", static_cast<unsigned long long>(t));
            return 6;
        }
        proto::ActuatorFrame ack;
        bool acked = false;
        const auto deadline = std::chrono::steady_clock::now() + 30s;
        while (!acked && std::chrono::steady_clock::now() < deadline) {
            proto::Message m;
            const auto got = channel.receive(m, 200ms);
            if (!got) {
                std::fprintf(stderr, "connection lost at tick %llu: %s\n", static_cast<unsigned long long>(t),
                             got.error().message.c_str());
                return 7;
            }
            if (!got.value()) continue;
            handle(m, &ack);
            acked = std::holds_alternative<proto::ActuatorFrame>(m);
        }
        if (!acked) {
            std::fprintf(stderr, "tick %llu not acknowledged\n", static_cast<unsigned long long>(t));
            return 8;
        }
        if (window.onAck(ack) != proto::AckVerdict::Completed || ack.result != proto::TickResult::Completed) {
            std::fprintf(stderr, "tick %llu ended with result %d\n", static_cast<unsigned long long>(t),
                         static_cast<int>(ack.result));
            return 9;
        }
        ++completed;
        lastLoops = ack.robotLoopCount;
    }
    const double wallMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    proto::SimControl stop;
    stop.command = proto::SimCommand::Stop;
    stop.commandId = 1;
    stop.sessionId = hs.sessionId;
    stop.runEpoch = hs.runEpoch;
    (void)channel.send(proto::Message(stop));
    for (int i = 0; i < 20; ++i) {
        proto::Message m;
        const auto got = channel.receive(m, 100ms);
        if (!got) break;
        if (got.value() && std::holds_alternative<proto::SimControlAck>(m)) break;
    }

    std::printf("SUMMARY ticks=%llu/%llu simTime=%.3fs wall=%.1fms robotLoops=%llu devices=%zu\n",
                static_cast<unsigned long long>(completed), static_cast<unsigned long long>(ticks),
                static_cast<double>(completed * opt.stepUs) * 1e-6, wallMs,
                static_cast<unsigned long long>(lastLoops), devices.size());
    if (opt.vision)
        std::printf("VISION_SUMMARY cameras=%zu framesSent=%llu cameraStates=%llu\n",
                    hello.value().ack.vision.cameras.size(),
                    static_cast<unsigned long long>(visionFramesSent),
                    static_cast<unsigned long long>(cameraStates));
    return completed == ticks ? 0 : 10;
}
