#include "client_e2e_support.h"

#include <cinttypes>

#include "wpiemu/core/match/own_predictor.h"

using namespace client_test;

namespace {

// ---- scripted drivers ---------------------------------------------------------------------------------------------------

struct Plan {
    // Robot-relative speeds as a function of the time since the drive started (seconds).
    static sdk::ChassisSpeeds slalom(double t) {
        if (t < 0.3) return {};
        if (t < 1.5) return {1.5f, 0.f, 0.f};
        if (t < 2.5) return {1.0f, 0.f, 1.0f};
        if (t < 3.5) return {0.f, 0.8f, 0.f};
        if (t < 4.5) return {-1.2f, 0.f, -0.6f};
        return {};
    }
    static sdk::ChassisSpeeds circle(double t) { return t < 0.3 ? sdk::ChassisSpeeds{} : sdk::ChassisSpeeds{1.0f, 0.f, 0.7f}; }
};

double yawDiff(double a, double b) { return wire::angleDifferenceRad(a, b); }

// What a run measures about one client's picture.
struct Series {
    E2eClient* c = nullptr;
    bool driving = false;
    double driveStartS = 0;
    double cmdAtS = -1, movedAtS = -1;
    std::vector<double> ownJumpMm, visualErrCm, remoteJumpMm, remoteErrCm, predErrCm;
    struct Pending {
        double tick;
        double x, y;
    };
    std::deque<Pending> pending;
    std::optional<Pose3> prevOwn;
    Vec3 prevOwnV;
    std::map<wire::ParticipantId, std::pair<Pose3, Vec3>> prevRemote;
    std::function<sdk::ChassisSpeeds(double)> plan;
};

class Recorder {
public:
    explicit Recorder(E2eWorld& w) : world_(w) {
        w.onSubStep = [this](E2eWorld& world) { sample(world); };
    }
    ~Recorder() { world_.onSubStep = nullptr; }

    Series& add(E2eClient& c, std::function<sdk::ChassisSpeeds(double)> plan = {}) {
        series_.push_back(std::make_unique<Series>());
        series_.back()->c = &c;
        series_.back()->plan = std::move(plan);
        return *series_.back();
    }
    // Commands the plans for the coming step (call before world.step()).
    void command(double nowS) {
        for (auto& s : series_) {
            if (!s->plan) continue;
            if (!s->driving) {
                s->driving = true;
                s->driveStartS = nowS;
            }
            const sdk::ChassisSpeeds speeds = s->plan(nowS - s->driveStartS);
            if (s->cmdAtS < 0 && (speeds.vx != 0.f || speeds.vy != 0.f || speeds.omega != 0.f)) s->cmdAtS = nowS;
            (void)s->c->client->driveRobotRelative(speeds);
        }
    }
    void drive(double seconds) {
        const int n = static_cast<int>(std::lround(seconds / 0.005));
        for (int i = 0; i < n; ++i) {
            command(world_.seconds());
            world_.step();
        }
    }
    Series& series(std::size_t i) { return *series_[i]; }

private:
    TruthPose truthLerp(double tick, wire::ParticipantId id) const {
        const std::uint64_t lo = static_cast<std::uint64_t>(std::floor(tick));
        const TruthPose a = world_.truthAt(lo, id), b = world_.truthAt(lo + 1, id);
        if (!a.valid) return {};
        if (!b.valid) return a;
        const double w = tick - static_cast<double>(lo);
        return {a.x + (b.x - a.x) * w, a.y + (b.y - a.y) * w, a.yaw, true};
    }

    void sample(E2eWorld& world) {
        for (auto& sp : series_) {
            Series& s = *sp;
            const auto view = s.c->client->view();
            if (!view || !view->hasOwn) continue;
            const double nowS = world.seconds();
            const double dt = 0.005 / std::max(1, world.options_.subSteps);

            if (view->own.predicted) {
                const Pose3& p = view->own.pose;
                if (s.prevOwn) {
                    const double dx = p.position.x - s.prevOwn->position.x, dy = p.position.y - s.prevOwn->position.y;
                    const double ex = 0.5 * (view->own.linearVelocity.x + s.prevOwnV.x) * dt, ey = 0.5 * (view->own.linearVelocity.y + s.prevOwnV.y) * dt;
                    s.ownJumpMm.push_back(std::hypot(dx - ex, dy - ey) * 1000.0);
                }
                s.prevOwn = p;
                s.prevOwnV = view->own.linearVelocity;
                if (view->diagnostics.predictedTick > 0)
                    s.pending.push_back({view->diagnostics.predictedTick, p.position.x, p.position.y});
                if (s.movedAtS < 0 && s.cmdAtS >= 0 && std::hypot(view->own.linearVelocity.x, view->own.linearVelocity.y) >= 0.05) s.movedAtS = nowS;
            }
            while (!s.pending.empty() && s.pending.front().tick + 1 <= static_cast<double>(world.hostTick())) {
                const TruthPose t = truthLerp(s.pending.front().tick, view->own.id);
                if (t.valid) s.visualErrCm.push_back(std::hypot(s.pending.front().x - t.x, s.pending.front().y - t.y) * 100.0);
                s.pending.pop_front();
            }
            if (view->diagnostics.snapshotsReceived > 20) s.predErrCm.push_back(view->diagnostics.predictionErrorM * 100.0);

            for (const mc::RemoteRobotView& r : view->remotes) {
                auto& prev = s.prevRemote[r.id];
                if (prev.first.position.x != 0.f || prev.first.position.y != 0.f) {
                    const double dx = r.pose.position.x - prev.first.position.x, dy = r.pose.position.y - prev.first.position.y;
                    const double ex = 0.5 * (r.linearVelocity.x + prev.second.x) * dt, ey = 0.5 * (r.linearVelocity.y + prev.second.y) * dt;
                    if (r.id != wire::kHostParticipantId) s.remoteJumpMm.push_back(std::hypot(dx - ex, dy - ey) * 1000.0);
                }
                prev = {r.pose, r.linearVelocity};
                if (r.id == wire::kHostParticipantId) continue;
                const double tick = static_cast<double>(view->renderSimTimeUs) / 5000.0;
                const TruthPose t = truthLerp(tick, r.id);
                if (t.valid && tick + 1 <= static_cast<double>(world.hostTick()))
                    s.remoteErrCm.push_back(std::hypot(r.pose.position.x - t.x, r.pose.position.y - t.y) * 100.0);
            }
        }
    }

    E2eWorld& world_;
    std::vector<std::unique_ptr<Series>> series_;
};

// The time the prediction alone needs to reach 0.05 m/s after an input at tick 0, in milliseconds.
double localRampMs() {
    PipelineKit kit;
    RobotConfig spec = swerveSpec("Ramp");
    auto kin = sdk::DriveKinematics::create(spec.drivetrain.spec).value();
    core::match::PredictionWorldArgs args{spec, nullptr, kit.drivetrains, kit.tires, {}, RobotStartPose{4.f, 4.f, 0.f}, 1, 60};
    auto world = core::match::PredictionWorld::create(args);
    REQUIRE(world.ok());
    core::match::OwnPredictor p(std::move(world).value(), {});
    core::match::OwnAuthoritative a;
    a.x = 4;
    a.y = 4;
    a.steerRad.assign(4, 0.0);
    a.spinRad.assign(4, 0.0);
    p.start(a, 0, 0, true);
    core::match::PredictedInput in;
    in.seq = 1;
    in.setpoints = kin.update({1.5f, 0.f, 0.f}).value();
    p.stepTick(&in, true);
    for (int i = 1; i < 200; ++i) {
        const Vec3 v = p.newest().linearVelocity;
        if (std::hypot(v.x, v.y) >= 0.05) return i * 5.0;
        p.stepTick(nullptr, true);
    }
    return -1;
}

void printRow(const char* what, const Stat& s, const char* unit) {
    std::printf("    %-34s n=%-6zu p50 %7.2f  p95 %7.2f  p99 %7.2f  max %7.2f %s\n", what, s.n, s.p50, s.p95, s.p99, s.max, unit);
}

// ---- measured results ---------------------------------------------------------------------------------------------------

struct DriveResult {
    std::string net;
    double ownLatencyMs = 0, rampMs = 0;
    Stat ownJump, visualErr, remoteJump, remoteErr, predErr, updateCost;
    mc::ClientDiagnostics diag;
    double finalCorrectionMm = 0;
    double finalErrCm = 0;
};

DriveResult measure(const Series& s, E2eWorld& world, wire::ParticipantId id) {
    DriveResult r;
    r.updateCost = statOf(s.c->updateUs);
    r.ownLatencyMs = (s.movedAtS - s.cmdAtS) * 1000.0;
    r.ownJump = statOf(s.ownJumpMm);
    r.visualErr = statOf(s.visualErrCm);
    r.remoteJump = statOf(s.remoteJumpMm);
    r.remoteErr = statOf(s.remoteErrCm);
    r.predErr = statOf(s.predErrCm);
    const auto view = s.c->client->view();
    r.diag = view->diagnostics;
    r.finalCorrectionMm = view->own.correctionM * 1000.0;
    const TruthPose t = world.truthAt(world.hostTick(), id);
    r.finalErrCm = t.valid ? std::hypot(view->own.pose.position.x - t.x, view->own.pose.position.y - t.y) * 100.0 : -1.0;
    return r;
}

void report(const char* title, const std::vector<DriveResult>& rows) {
    if (!reportRequested()) return;
    std::printf("\n== %s\n", title);
    for (const DriveResult& r : rows) {
        const mc::ClientDiagnostics& d = r.diag;
        std::printf("  %s\n    rtt %.1f ms (min %.1f)  snapshots %.1f Hz  loss %.0f permille  buffer %.1f ms  lead %d ticks (host saw %d)  ahead %.1f ms\n"
                    "    reconciliations %" PRIu64 " (snaps %" PRIu64 ", %" PRIu64 " ticks replayed)  late inputs %" PRIu64 "  resyncs %" PRIu64 "\n",
                    r.net.c_str(), d.rttMs, d.rttMinMs, d.snapshotRateHz, d.lossPermille, d.bufferDelayMs, d.inputLeadTicks, d.hostReportedLeadTicks,
                    d.predictionAheadMs, d.reconciliations, d.snaps, d.replayedTicks, d.lateInputs, d.resyncs);
        printRow("own pose jump per frame", r.ownJump, "mm");
        printRow("own picture vs host truth", r.visualErr, "cm");
        printRow("prediction error (vs snapshot)", r.predErr, "cm");
        printRow("remote robot jump per frame", r.remoteJump, "mm");
        printRow("remote picture vs host truth", r.remoteErr, "cm");
        printRow("MatchClient::update() wall time", r.updateCost, "us");
        std::printf("    input to visible motion %.1f ms (local ramp %.1f ms, extra %.1f ms)   at rest: error %.2f cm, correction left %.2f mm\n",
                    r.ownLatencyMs, r.rampMs, r.ownLatencyMs - r.rampMs, r.finalErrCm, r.finalCorrectionMm);
    }
}

// Two clients drive for `seconds` over `net`; returns what each saw of its own robot and of the other.
std::vector<DriveResult> driveTwo(const NetProfile& net, double seconds, double settleS = 1.5) {
    E2eWorld world;
    E2eClient& a = world.addClient("A", net, E2eWorld::robotSpec("A"));
    E2eClient& b = world.addClient("B", net, E2eWorld::robotSpec("B"));
    world.run(2.0);
    Recorder rec(world);
    rec.add(a, Plan::slalom);
    rec.add(b, Plan::circle);
    rec.drive(seconds);
    // Both stop; the pictures must settle on the host's truth.
    rec.series(0).plan = [](double) { return sdk::ChassisSpeeds{}; };
    rec.series(1).plan = [](double) { return sdk::ChassisSpeeds{}; };
    rec.drive(settleS);
    const double ramp = localRampMs();
    std::vector<DriveResult> out;
    for (std::size_t i = 0; i < 2; ++i) {
        DriveResult r = measure(rec.series(i), world, rec.series(i).c->id());
        r.net = net.name + (i == 0 ? ", client A (slalom)" : ", client B (circle)");
        r.rampMs = ramp;
        out.push_back(std::move(r));
    }
    return out;
}

}  // namespace

TEST_CASE("client e2e: on a LAN the own robot is exact, remote robots interpolate smoothly") {
    const std::vector<DriveResult> rows = driveTwo(lanProfile(), 7.0);
    report("LAN 60 Hz", rows);
    for (const DriveResult& r : rows) {
        CHECK(r.ownLatencyMs <= r.rampMs + 6.0);          // no network in the first reaction of the own robot
        CHECK(r.ownJump.max < 8.0);                       // mm per 5 ms frame beyond what the velocity explains
        CHECK(r.visualErr.p95 < 1.5);                     // cm against the host's truth at the same tick (the two robots may touch)
        CHECK(r.visualErr.max < 5.0);
        CHECK(r.predErr.p95 < 1.5);
        CHECK(r.remoteJump.max < 3.0);
        CHECK(r.remoteErr.p95 < 0.5);
        CHECK(r.diag.bufferDelayMs > 30.0);               // about 35-40 ms on a LAN: two snapshot intervals and a little jitter
        CHECK(r.diag.bufferDelayMs < 50.0);
        CHECK(r.diag.snaps == 0);
        CHECK(r.diag.lateInputs <= 3);
        CHECK(r.finalErrCm < 2.5);
        CHECK(r.finalCorrectionMm < 10.0);                // bounded like the picture error above; the Linux scheduler leaves a few mm
    }
}

TEST_CASE("client e2e: on a WAN prediction keeps the own robot exact and the interpolation buffer grows") {
    const std::vector<DriveResult> rows = driveTwo(wanProfile(), 7.0);
    report("WAN 30 Hz, 60 ms RTT, 20 ms jitter, 3 percent loss", rows);
    for (const DriveResult& r : rows) {
        CHECK(r.ownLatencyMs <= r.rampMs + 6.0);
        CHECK(r.ownJump.max < 10.0);
        CHECK(r.visualErr.p95 < 2.5);
        CHECK(r.visualErr.max < 10.0);
        CHECK(r.remoteJump.max < 8.0);
        CHECK(r.remoteErr.p95 < 1.0);
        CHECK(r.diag.bufferDelayMs > 55.0);               // about 60-80 ms
        CHECK(r.diag.bufferDelayMs < 130.0);
        CHECK(r.diag.snapshotRateHz > 25.0);
        CHECK(r.diag.snapshotRateHz < 35.0);
        CHECK(r.diag.snaps == 0);
        CHECK(r.finalErrCm < 2.5);
        CHECK(r.finalCorrectionMm < 10.0);
    }
}

TEST_CASE("client e2e: on a bad WAN with loss and reordering nothing pops and the picture still converges") {
    const std::vector<DriveResult> rows = driveTwo(badWanProfile(), 7.0, 4.0);
    report("bad WAN 150 ms RTT, 8 percent loss, reordering", rows);
    for (const DriveResult& r : rows) {
        CHECK(r.ownLatencyMs <= r.rampMs + 6.0);
        CHECK(r.ownJump.max < 25.0);
        CHECK(r.visualErr.p95 < 5.0);
        CHECK(r.visualErr.max < 20.0);
        CHECK(r.remoteJump.max < 40.0);                   // a lost snapshot is bridged by extrapolation, not a pop
        CHECK(r.remoteErr.p95 < 2.0);
        CHECK(r.diag.bufferDelayMs > 60.0);
        CHECK(r.diag.bufferDelayMs <= 250.0);
        CHECK(r.diag.snaps == 0);
        CHECK(r.finalErrCm < 5.0);                        // 8 % loss and reordering: a late input is still being absorbed
        CHECK(r.finalCorrectionMm < 12.0);
        CHECK(r.diag.lateInputs <= 5);
    }
}

namespace {

// Puts the host's own robot (no robot program drives it) at `position` facing `yaw`, at rest.
void placeHostRobot(E2eWorld& world, double x, double y, double yaw) {
    const RobotView* host = world.rig.session->robots().find(wire::kHostParticipantId);
    REQUIRE(host != nullptr);
    Pose3 p = world.rig.session->world().pose(host->body);
    p.position.x = static_cast<float>(x);
    p.position.y = static_cast<float>(y);
    p.rotation = Quat::fromYaw(static_cast<float>(yaw));
    world.rig.session->world().setPose(host->body, p);
    world.rig.session->world().setLinearVelocity(host->body, {});
    world.rig.session->world().setAngularVelocity(host->body, {});
}

Pose3 hostPose(E2eWorld& world, wire::ParticipantId id) {
    const RobotView* v = world.rig.session->robots().find(id);
    REQUIRE(v != nullptr);
    return world.rig.session->world().pose(v->body);
}

}  // namespace

TEST_CASE("client e2e: a collision with the host's robot is corrected smoothly, never as a teleport") {
    std::vector<DriveResult> rows;
    for (const NetProfile& net : {lanProfile(), wanProfile()}) {
        E2eWorld world;
        E2eClient& a = world.addClient("A", net, E2eWorld::robotSpec("A"));
        world.run(1.5);
        const Pose3 start = a.client->view()->own.pose;
        const double yaw = start.rotation.yaw();
        placeHostRobot(world, start.position.x + 1.3 * std::cos(yaw), start.position.y + 1.3 * std::sin(yaw), yaw + 3.14159265);
        const Pose3 hostBefore = hostPose(world, wire::kHostParticipantId);
        Recorder rec(world);
        rec.add(a, [](double t) { return t < 0.2 ? sdk::ChassisSpeeds{} : (t < 2.5 ? sdk::ChassisSpeeds{1.5f, 0.f, 0.f} : sdk::ChassisSpeeds{}); });
        rec.drive(2.8);
        rec.series(0).plan = [](double) { return sdk::ChassisSpeeds{}; };
        rec.drive(2.0);
        DriveResult r = measure(rec.series(0), world, a.id());
        r.net = net.name + ", pushing the host's robot";
        r.rampMs = localRampMs();
        const Pose3 hostAfter = hostPose(world, wire::kHostParticipantId);
        const double pushedM = std::hypot(hostAfter.position.x - hostBefore.position.x, hostAfter.position.y - hostBefore.position.y);
        if (reportRequested()) std::printf("  host robot pushed %.2f cm, reconciliations %" PRIu64 ", ticks replayed %" PRIu64 "\n", pushedM * 100.0, r.diag.reconciliations, r.diag.replayedTicks);

        CHECK(pushedM > 0.05);                      // the contact really happened in the host's world
        CHECK(r.diag.reconciliations >= 1);          // and the client had to be corrected
        CHECK(r.diag.snaps == 0);
        CHECK(r.ownJump.max < 25.0);                // mm in one 5 ms frame: no teleport, the offset is absorbed over ~100 ms
        CHECK(r.updateCost.p99 < 10'000.0);         // even a replay of 20-60 ticks fits a frame (microseconds of wall time)
        CHECK(r.finalErrCm < (net.rttMs < 10 ? 3.0 : 5.0));   // the picture converged on the host's truth (the robots stay in contact)
        CHECK(r.finalCorrectionMm < 10.0);
        rows.push_back(std::move(r));
    }
    report("collision with the host robot", rows);
}

TEST_CASE("client e2e: clock sync converges under asymmetric delay and inputs still arrive in time") {
    struct Case {
        NetProfile net;
        double expectedBiasMs;   // the estimate of the host's sim time minus the truth: (up - down) / 2, and 5 ms for the host
                                 // answering a ping before its tick of that step
    };
    const std::vector<Case> cases = {{asymmetricProfile(10'000, 50'000), -25.0}, {asymmetricProfile(50'000, 10'000), 15.0},
                                     {lanProfile(), -5.0}};
    for (const Case& c : cases) {
        E2eWorld world;
        E2eClient& a = world.addClient("A", c.net, E2eWorld::robotSpec("A"));
        Recorder rec(world);
        rec.add(a, Plan::slalom);
        std::vector<double> errorsMs;
        double early = 0;
        for (int i = 0; i < 1800; ++i) {
            rec.command(world.seconds());
            world.step();
            const auto view = a.client->view();
            if (!view->diagnostics.clockSynced) continue;
            const double err = static_cast<double>(view->diagnostics.hostSimEstimateUs - world.rig.session->clock().simTimeUs()) / 1000.0;
            if (i == 300) early = err;
            if (i >= 1200) errorsMs.push_back(err);
        }
        const Stat s = statOf(errorsMs);
        const auto d = a.client->view()->diagnostics;
        double lo = errorsMs.front(), hi = errorsMs.front();
        for (const double e : errorsMs) {
            lo = std::min(lo, e);
            hi = std::max(hi, e);
        }
        if (reportRequested())
            std::printf("  %-28s clock error: at 1.5 s %.1f ms, last 3 s mean %.1f ms (range %.1f .. %.1f), expected bias %.1f ms, bound +-%.1f ms; lead %d ticks, host saw %d, late %" PRIu64 "\n",
                        c.net.name.c_str(), early, s.mean, lo, hi, c.expectedBiasMs, d.clockAsymmetryBoundMs, d.inputLeadTicks, d.hostReportedLeadTicks, d.lateInputs);
        CHECK(std::fabs(s.mean - c.expectedBiasMs) < 4.0);            // the bias is half the asymmetry, no more
        CHECK(std::fabs(s.mean) <= d.clockAsymmetryBoundMs + 6.0);    // and inside the bound the minimum RTT gives
        CHECK(hi - lo < 3.0);                                         // converged: stable to a few milliseconds
        CHECK(std::fabs(early - s.mean) < 6.0);                       // and it was there after 1.5 s already
        // The lead follows what the host measures, not the (biased) clock: inputs arrive before their tick.
        CHECK(d.hostReportedLeadTicks >= 1);
        CHECK(d.lateInputs <= 5);
        CHECK(statOf(rec.series(0).visualErrCm).p95 < 1.0);
    }
}

TEST_CASE("client e2e: a robot beyond the fairness limits is clamped, told so, and still predicted like the host simulates it") {
    E2eWorld world;
    RobotConfig big = E2eWorld::robotSpec("Big");
    big.chassis.massKg = 500.f;
    big.chassis.sizeM = {2.4f, 2.4f, 0.4f};
    big.chassis.comHeightM = 0.3f;
    E2eClient& a = world.addClient("Big", lanProfile(), std::move(big));
    const auto view = a.client->view();
    CHECK_FALSE(view->specClamps.empty());
    const auto accepted = a.client->acceptedSpec();
    REQUIRE(accepted);
    CHECK(accepted->chassis.massKg == doctest::Approx(static_cast<float>(sdk::kRemoteSpecLimits.massKg.max)));
    CHECK(accepted->chassis.sizeM.x < 2.4f);
    // The host accepted the same robot: what it tells the other players to draw is the clamped size.
    const auto participants = world.h.host->participants();
    REQUIRE(participants.size() == 1);
    REQUIRE(participants[0].appearance);
    CHECK(participants[0].appearance->sizeXMm == wire::quantizeMilliU16(accepted->chassis.sizeM.x));
    CHECK(participants[0].appearance->sizeYMm == wire::quantizeMilliU16(accepted->chassis.sizeM.y));

    world.run(1.0);
    Recorder rec(world);
    rec.add(a, Plan::slalom);
    rec.drive(6.0);
    const DriveResult r = measure(rec.series(0), world, a.id());
    report("clamped robot", {r});
    CHECK(r.visualErr.p95 < 1.0);                   // predicted with the clamped spec, so it follows the host's robot (0.5-0.65 cm measured)
    CHECK(r.diag.snaps == 0);
}

TEST_CASE("client e2e: the host stalling or pausing freezes the picture and the match resumes cleanly") {
    for (const wire::MatchState stall : {wire::MatchState::HostStalled, wire::MatchState::Paused}) {
        E2eWorld world;
        E2eClient& a = world.addClient("A", wanProfile(), E2eWorld::robotSpec("A"));
        E2eClient& b = world.addClient("B", wanProfile(), E2eWorld::robotSpec("B"));
        Recorder rec(world);
        const auto towardMiddle = [](E2eClient& c) { return c.client->view()->own.pose.position.y < 4.1 ? 0.5f : -0.5f; };
        const float vyA = towardMiddle(a), vyB = towardMiddle(b);
        rec.add(a, [vyA](double t) { return t < 0.3 ? sdk::ChassisSpeeds{} : sdk::ChassisSpeeds{0.f, vyA, 0.f}; });
        rec.add(b, [vyB](double t) { return t < 0.3 ? sdk::ChassisSpeeds{} : sdk::ChassisSpeeds{0.f, vyB, 0.f}; });
        rec.drive(2.0);
        CHECK(a.state() == mc::ConnectionState::Live);

        world.h.obs.matchState = stall;   // the host stops its world
        rec.drive(1.2);
        const mc::ConnectionState expected = stall == wire::MatchState::Paused ? mc::ConnectionState::Paused : mc::ConnectionState::HostStalled;
        CHECK(a.state() == expected);
        CHECK(b.state() == expected);
        CHECK(a.client->view()->hud.state == stall);
        const Pose3 own = a.client->view()->own.pose;
        const mc::RemoteRobotView* other = nullptr;
        const auto frozenView = a.client->view();
        for (const mc::RemoteRobotView& r : frozenView->remotes)
            if (r.id == b.id()) other = &r;
        REQUIRE(other != nullptr);
        const Pose3 otherPose = other->pose;
        rec.drive(0.6);
        const auto later = a.client->view();
        CHECK(std::hypot(later->own.pose.position.x - own.position.x, later->own.pose.position.y - own.position.y) < 0.002);   // frozen
        for (const mc::RemoteRobotView& r : later->remotes)
            if (r.id == b.id()) CHECK(std::hypot(r.pose.position.x - otherPose.position.x, r.pose.position.y - otherPose.position.y) < 0.002);

        world.h.obs.matchState = wire::MatchState::Running;
        rec.series(0).movedAtS = -1;
        rec.drive(2.5);
        CHECK(a.state() == mc::ConnectionState::Live);
        CHECK(b.state() == mc::ConnectionState::Live);
        const auto after = a.client->view();
        CHECK(std::hypot(after->own.pose.position.x - later->own.pose.position.x, after->own.pose.position.y - later->own.pose.position.y) > 0.8);   // moving again
        const DriveResult r = measure(rec.series(0), world, a.id());
        report(stall == wire::MatchState::Paused ? "paused and resumed" : "host stalled and resumed", {r});
        CHECK(r.visualErr.p95 < 2.0);
        CHECK(r.diag.snaps == 0);
        CHECK(r.diag.clockSynced);
    }
}

TEST_CASE("client e2e: after a dropped link the client reconnects with its token and plays on as the same participant") {
    E2eWorld world;
    E2eClient& a = world.addClient("A", wanProfile(), E2eWorld::robotSpec("A"));
    E2eClient& b = world.addClient("B", wanProfile(), E2eWorld::robotSpec("B"));
    Recorder rec(world);
    rec.add(a, [](double t) { return t < 0.3 ? sdk::ChassisSpeeds{} : sdk::ChassisSpeeds{0.5f, 0.f, 0.f}; });
    rec.add(b, Plan::circle);
    rec.drive(2.0);
    const wire::ParticipantId idBefore = a.id();
    const proto::AllianceStation stationBefore = a.client->view()->own.station;
    CHECK(world.h.host->stats().joinsAccepted == 2);
    CHECK(world.h.host->stats().resumes == 0);

    a.cutLink();
    bool sawReconnecting = false, sawLiveAgain = false;
    int framesToLive = 0;
    for (int i = 0; i < 2000 && !sawLiveAgain; ++i) {
        rec.command(world.seconds());
        world.step();
        const mc::ConnectionStatus s = a.client->status();
        if (s.state == mc::ConnectionState::Connecting && s.reconnecting) sawReconnecting = true;
        if (sawReconnecting && s.state == mc::ConnectionState::Live) sawLiveAgain = true;
        ++framesToLive;
    }
    CHECK(sawReconnecting);
    REQUIRE(sawLiveAgain);
    if (reportRequested()) std::printf("  reconnected after %.2f s\n", framesToLive * 0.005);
    CHECK(framesToLive * 0.005 < 3.0);
    CHECK(a.id() == idBefore);
    CHECK(a.client->view()->own.station == stationBefore);
    CHECK(world.h.host->stats().resumes == 1);
    CHECK(world.h.host->stats().joinsAccepted == 2);   // no new participant
    CHECK(world.h.host->joinedCount() == 2);

    // It keeps driving, and the picture follows the host's robot again once the new session is in.
    rec.series(0).visualErrCm.clear();
    rec.series(0).pending.clear();
    rec.drive(4.0);
    const DriveResult r = measure(rec.series(0), world, a.id());
    report("after a reconnect", {r});
    CHECK(a.state() == mc::ConnectionState::Live);
    CHECK(r.visualErr.p95 < 2.0);
    CHECK(r.diag.snaps <= 1);
    CHECK(r.finalErrCm < 2.0);
    // The other player never lost sight of it.
    bool listed = false;
    for (const mc::RemoteRobotView& rr : b.client->view()->remotes)
        if (rr.id == idBefore) listed = true;
    CHECK(listed);
}

TEST_CASE("client e2e: two clients and the host see one world: robots, wheels, zones, fuel and the match state agree") {
    E2eWorld::Options options;
    options.neutralFuel = 120;
    E2eWorld world(options);
    E2eClient& a = world.addClient("A", lanProfile(), E2eWorld::robotSpec("A", true));
    E2eClient& b = world.addClient("B", lanProfile(), E2eWorld::robotSpec("B", true));
    world.run(1.0);
    const wire::ParticipantId idA = a.id(), idB = b.id();
    REQUIRE(idA != idB);

    // B moves its shooter zone (zone 1) and switches the pickup (zone 0) on.
    REQUIRE(b.client->setZonePose(1, {{0.12f, 0.05f, 0.45f}, Quat::fromEuler(0.f, -0.3f, 0.f)}).ok());
    b.client->setZoneActive(0, true);

    Recorder rec(world);
    rec.add(a, Plan::slalom);
    rec.add(b, Plan::circle);
    rec.drive(6.0);
    rec.series(0).plan = rec.series(1).plan = [](double) { return sdk::ChassisSpeeds{}; };
    rec.drive(3.0);

    const auto viewA = a.client->view();
    const auto viewB = b.client->view();
    const auto truthSnapshot = world.rig.snapshot();
    const auto hostPoseOf = [&](wire::ParticipantId id) { return world.truthAt(world.hostTick(), id); };

    for (const auto& [view, self, other] : {std::tuple{viewA, idA, idB}, std::tuple{viewB, idB, idA}}) {
        // Who is in the match.
        std::set<wire::ParticipantId> seen;
        for (const mc::RemoteRobotView& r : view->remotes) {
            seen.insert(r.id);
            if (r.id != wire::kHostParticipantId) {
                REQUIRE(r.appearance);                   // everybody can be drawn (this rig's host has no wheels to describe)
                CHECK(r.appearance->wheels.size() == 4);
            }
            CHECK_FALSE(r.name.empty());
            const TruthPose t = hostPoseOf(r.id);
            REQUIRE(t.valid);
            CHECK(std::hypot(r.pose.position.x - t.x, r.pose.position.y - t.y) < 0.015);
            CHECK(std::fabs(yawDiff(r.pose.rotation.yaw(), t.yaw)) < 0.01);
            // The wheels point where the host's wheels point.
            const sdk::RobotSample* sample = g26session::Rig::sampleOf(*truthSnapshot, r.id);
            if (r.id != wire::kHostParticipantId) {
                REQUIRE(sample != nullptr);
                REQUIRE(r.wheels.size() == sample->wheels.size());
                for (std::size_t i = 0; i < r.wheels.size(); ++i)
                    CHECK(std::fabs(yawDiff(r.wheels[i].steerRad, sample->wheels[i].steerAngleRad)) < 0.03);
            }
        }
        CHECK(seen == std::set<wire::ParticipantId>{wire::kHostParticipantId, other});
        CHECK(view->own.id == self);
        const TruthPose own = hostPoseOf(self);
        CHECK(std::hypot(view->own.pose.position.x - own.x, view->own.pose.position.y - own.y) < 0.015);
        CHECK(view->roster.size() == 3);
        // The match state is the host's.
        CHECK(view->hud.state == wire::MatchState::Running);
        CHECK(view->hud.enabled);
        CHECK_FALSE(view->hud.game.empty());
        CHECK(view->connection.state == mc::ConnectionState::Live);
    }

    // B's moved zone: B sees it, A sees it, and the host simulates it.
    const auto zoneOfB = [&](const mc::MatchClientView& v) -> const mc::RemoteRobotView* {
        for (const mc::RemoteRobotView& r : v.remotes)
            if (r.id == idB) return &r;
        return nullptr;
    };
    const mc::RemoteRobotView* bAsSeenByA = zoneOfB(*viewA);
    REQUIRE(bAsSeenByA != nullptr);
    REQUIRE(bAsSeenByA->zonePoses.size() == 2);
    CHECK((bAsSeenByA->zoneFlags & 1u) == 1u);
    for (const sdk::Pose3* p : {&bAsSeenByA->zonePoses[1], &viewB->own.zonePoses[1]}) {
        CHECK(p->position.x == doctest::Approx(0.12f).epsilon(0.02));
        CHECK(p->position.y == doctest::Approx(0.05f).epsilon(0.05));
        CHECK(p->position.z == doctest::Approx(0.45f).epsilon(0.02));
    }
    const auto info = world.adapter.remoteRobotInfo(idB);
    REQUIRE(info.has_value());
    REQUIRE(info->zonePoses.size() == 2);
    CHECK(info->zonePoses[1].position.x == doctest::Approx(0.12f).epsilon(0.01));
    CHECK((info->zoneFlags & 1u) == 1u);
    CHECK((viewB->own.zoneFlags & 1u) == 1u);

    // The fuel both clients hold is the fuel the host has.
    const sdk::EntityGroupSample* fuel = nullptr;
    for (const sdk::EntityGroupSample& g : truthSnapshot->entityGroups())
        if (g.group == "fuel") fuel = &g;
    REQUIRE(fuel != nullptr);
    REQUIRE(fuel->ids.size() > 50);
    for (const auto& view : {viewA, viewB}) {
        std::map<std::uint32_t, Vec3> held;
        for (const mc::EntityGroupView& g : view->entities)
            for (const mc::EntityView& e : g.entities) held[e.id] = {e.x, e.y, e.z};
        std::size_t missing = 0, off = 0;
        for (std::size_t i = 0; i < fuel->ids.size(); ++i) {
            const auto it = held.find(fuel->ids[i]);
            if (it == held.end()) {
                ++missing;
                continue;
            }
            const Vec3 d = it->second - fuel->positions[i];
            if (std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z) > 0.06f) ++off;
        }
        if (reportRequested())
            std::printf("  fuel: host %zu, client holds %zu, missing %zu, more than 6 cm off %zu\n", fuel->ids.size(), held.size(), missing, off);
        CHECK(missing <= fuel->ids.size() / 50);
        CHECK(off <= fuel->ids.size() / 20);
        CHECK(held.size() <= fuel->ids.size() + fuel->ids.size() / 50);
    }
}

TEST_CASE("client e2e: five clients and the host on a WAN, the full stress run") {
    if (!fullRunRequested()) return;
    E2eWorld::Options options;
    options.neutralFuel = 200;
    E2eWorld world(options);
    std::vector<E2eClient*> clients;
    for (int i = 0; i < 5; ++i)
        clients.push_back(&world.addClient(std::string("P") + std::to_string(i + 1), wanProfile(), E2eWorld::robotSpec("P", true)));
    Recorder rec(world);
    for (std::size_t i = 0; i < clients.size(); ++i)
        rec.add(*clients[i], [i](double t) { return Plan::slalom(std::fmod(t + 0.9 * static_cast<double>(i), 6.0)); });
    rec.drive(20.0);
    std::vector<DriveResult> rows;
    for (std::size_t i = 0; i < clients.size(); ++i) {
        DriveResult r = measure(rec.series(i), world, clients[i]->id());
        r.net = "player " + std::to_string(i + 1) + " of 5 on the WAN";
        r.rampMs = localRampMs();
        rows.push_back(std::move(r));
    }
    report("five clients and the host, 20 s", rows);
    for (const DriveResult& r : rows) {
        CHECK(r.visualErr.p95 < 8.0);
        CHECK(r.remoteJump.max < 150.0);
        CHECK(r.diag.snaps <= 10);
    }
}
