// The client app against a REAL practice-match host in this process: the 2026 session over Box3D, MatchHosting with the invite
// and the SecureTransport on every connection, a loopback ENet link, and the client app's controller driving it with a fake
// joystick (no window). Encryption is on throughout: the client only reaches the match through the invite handshake.
#include <doctest/doctest.h>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QTemporaryDir>

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

#include "client/client_controller.h"
#include "game_2026/multirobot_session_rig.h"
#include "host/match_hosting.h"
#include "match/match_test_support.h"
#include "wpiemu/net/invite.h"
#include "wpiemu/plugins/game_2026/plugin.h"
#include "wpiemu/sdk/testing/wait.h"

namespace {

using namespace std::chrono_literals;
using namespace wpiemu;
namespace host = wpiemu::app::host;
namespace cl = wpiemu::app::client;
namespace mc = wpiemu::core::match;
namespace wire = wpiemu::sdk::match;
namespace rt = wpiemu::core::runtime;

// The sim thread of a real runner: poll, tick at 200 Hz, tick end.
class SimLoop {
public:
    SimLoop(g26session::Rig& rig, host::MatchHosting& hosting) : rig_(rig), hosting_(hosting) {}
    ~SimLoop() { stop(); }
    void start() {
        thread_ = std::thread([this] {
            auto next = std::chrono::steady_clock::now();
            while (!stop_.load()) {
                const rt::RunnerObservation obs{wire::MatchState::Running, 1};
                for (int i = 0; i < 8 && std::chrono::steady_clock::now() >= next; ++i, next += 5ms) {
                    hosting_.onPoll(obs);
                    rig_.session->tick(rig_.frame, false);
                    hosting_.onTickEnd(obs);
                }
                std::this_thread::sleep_for(1ms);
            }
        });
    }
    void stop() {
        stop_.store(true);
        if (thread_.joinable()) thread_.join();
    }

private:
    g26session::Rig& rig_;
    host::MatchHosting& hosting_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
};

class FakeInput final : public cl::IClientInput {
public:
    sdk::proto::JoystickState sample() override { return state; }
    sdk::proto::JoystickState state = [] {
        sdk::proto::JoystickState s;
        s.present = true;
        s.axes = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
        s.povs = {-1};
        s.buttonCount = 12;
        return s;
    }();
    void forward(float amount) { state.axes[cl::kForwardAxis] = -amount; }
};

// The host side of one match.
struct HostWorld {
    g26session::Rig rig{[] {
        g26session::Options o;
        o.neutralFuel = 30;
        o.depotFuel = 0;
        return o;
    }()};
    match_test::PipelineKit kit;
    mc::SimulationMatchSession adapter{*rig.session};
    std::unique_ptr<host::MatchHosting> hosting;
    std::unique_ptr<SimLoop> loop;
    std::string invite;

    HostWorld() {
        REQUIRE(wpiemu::plugins::game_2026::registerPlugin(kit.plugins).ok());
        host::HostingOptions options;
        options.mode = host::HostingMode::Direct;
        options.bindHost = "127.0.0.1";
        options.port = 0;
        host::MatchHostingServices services;
        services.session = &adapter;
        services.pipeline = &kit.pipeline;
        auto created = host::MatchHosting::create(options, std::move(services));
        REQUIRE_MESSAGE(created.ok(), (created.ok() ? "" : created.error().message));
        hosting = std::move(created).value();
        invite = hosting->status()->invite;
        hosting->start();
        REQUIRE(sdk::testing::waitUntil([&] { return hosting->status()->accepting(); }, 5s));
        loop = std::make_unique<SimLoop>(rig, *hosting);
        loop->start();
    }
    ~HostWorld() {
        if (loop) loop->stop();
        if (hosting) hosting->shutdown();
    }

    std::string address() const {
        std::string endpoint = hosting->status()->endpoint;   // enet://127.0.0.1:PORT
        return endpoint.substr(std::string("enet://").size());
    }
};

std::string robotJson(bool zones) {
    sdk::RobotConfig spec = zones ? g26session::Rig::fuelRobot() : match_test::swerveSpec("Client Bot");
    spec.name = "Client Bot";
    spec.gamePluginId = std::string(wpiemu::plugins::game_2026::pluginId());
    return match_test::specJson(spec);
}

QString writeRobot(QTemporaryDir& dir, bool zones) {
    const QString path = dir.filePath(QStringLiteral("robot.json"));
    QFile file(path);
    REQUIRE(file.open(QIODevice::WriteOnly));
    file.write(QByteArray::fromStdString(robotJson(zones)));
    return path;
}

template <class Pred>
bool tickUntil(cl::ClientController& controller, Pred pred, std::chrono::milliseconds limit = 10s) {
    const auto deadline = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < deadline) {
        controller.tick(0.016);
        QCoreApplication::processEvents();
        if (pred()) return true;
        std::this_thread::sleep_for(16ms);
    }
    return false;
}

bool isLive(const cl::ClientController& c) {
    return c.presentation().screen == cl::Screen::Drive && c.presentation().banner == cl::Banner::None && c.presentation().controlsEnabled;
}

double distance(const sdk::Vec3& a, const sdk::Vec3& b) { return std::hypot(b.x - a.x, b.y - a.y); }

}  // namespace

TEST_CASE("client e2e: join over the invite handshake, go live, drive the own robot, move a zone, leave") {
    HostWorld world;
    QTemporaryDir dir;
    REQUIRE(dir.isValid());
    const QString robotPath = writeRobot(dir, true);

    auto services = cl::ClientServices::create().value();
    app::MemoryKeyValueStore store;
    FakeInput input;
    cl::ClientController controller(*services, store, &input);

    controller.chooseRobot(robotPath);
    REQUIRE_MESSAGE(controller.robot().ok, (controller.robot().errors.empty() ? "" : controller.robot().errors.front()));
    CHECK(controller.robot().zoneCount == 2);

    cl::JoinFormInput form;
    form.invite = world.invite;
    form.directAddress = world.address();
    form.displayName = "Iron Owls";
    form.robotPath = robotPath.toStdString();
    const cl::JoinValidation joined = controller.join(form);
    REQUIRE(joined.ok());
    CHECK(controller.presentation().screen == cl::Screen::Join);   // the form stays up until the host accepts

    REQUIRE_MESSAGE(tickUntil(controller, [&] { return isLive(controller); }),
                    "never went live: banner " << static_cast<int>(controller.presentation().banner) << " "
                                               << controller.presentation().detail);
    REQUIRE(controller.view());
    const mc::MatchClientView& view = *controller.view();
    CHECK(view.hasOwn);
    CHECK(view.own.name == "Iron Owls");
    CHECK(view.own.appearance != nullptr);
    CHECK(view.remotes.size() == 1);   // the host's own robot
    CHECK(view.remotes[0].isHost);
    CHECK(controller.snapshot() != nullptr);
    CHECK(controller.sceneRevision() > 0);
    REQUIRE(controller.roster());
    CHECK(controller.roster()->size() == 1);
    CHECK_FALSE(controller.scene().hud.empty());   // the 2026 game's HUD widgets came through the plugin aggregator
    CHECK(controller.scene().field != nullptr);

    // The host only ever saw a client that proved it holds the invite.
    CHECK(world.hosting->status()->joinsAccepted == 1);
    CHECK(world.hosting->status()->handshakesFailed == 0);

    // Drive forward: the predicted own robot moves at once and the host's snapshots confirm it.
    REQUIRE(tickUntil(controller, [&] { return controller.view() && controller.view()->diagnostics.snapshotsReceived > 20; }));
    CHECK(controller.view()->own.predicted);   // the prediction world (field, own chassis, the shared wheel controller) is running
    const sdk::Vec3 before = controller.view()->own.pose.position;
    input.forward(1.f);
    REQUIRE_MESSAGE(tickUntil(controller, [&] { return distance(before, controller.view()->own.pose.position) > 0.5; }, 8s),
                    "the own robot did not move: " << distance(before, controller.view()->own.pose.position) << " m");
    CHECK(std::isfinite(controller.view()->diagnostics.rttMs));
    CHECK(controller.view()->diagnostics.snapshotAgeMs < 1000.0);

    // The robot is still driving and the host's own picture of it moves too (its participant is in the host's roster).
    CHECK(world.hosting->status()->roster.size() == 2);
    input.forward(0.f);

    // Zone buttons and the zone pose sliders reach the host through the same input stream.
    controller.setZoneButton(0, true);
    REQUIRE(tickUntil(controller, [&] { return (controller.view()->own.zoneFlags & 1u) != 0; }));
    controller.setZoneButton(0, false);
    const sdk::Vec3 rest = controller.zonePoses().restPose(1).position;
    controller.moveZone(1, {rest.x + 0.1f, rest.y + 0.1f, rest.z}, 0.f);
    REQUIRE(tickUntil(controller, [&] {
        const auto& poses = controller.view()->own.zonePoses;
        return poses.size() == 2 && std::fabs(poses[1].position.y - (rest.y + 0.1f)) < 0.02f;
    }));
    controller.resetZone(1);
    CHECK_FALSE(controller.zonePoses().moved(1));

    controller.leave();
    REQUIRE(tickUntil(controller, [&] { return controller.sessionState().phase == cl::LinkPhase::Idle; }, 5s));
    controller.tick(0.016);
    CHECK(controller.presentation().screen == cl::Screen::Join);
    CHECK(controller.view() == nullptr);
    REQUIRE(sdk::testing::waitUntil([&] { return world.hosting->status()->remoteCount() == 0; }, 5s));
}

TEST_CASE("client e2e: a wrong invite never joins and the form says why") {
    HostWorld world;
    QTemporaryDir dir;
    const QString robotPath = writeRobot(dir, false);
    auto services = cl::ClientServices::create().value();
    app::MemoryKeyValueStore store;
    FakeInput input;
    cl::ClientController controller(*services, store, &input);

    cl::JoinFormInput form;
    form.invite = net::inviteText(net::generateInvite().value());   // well formed, but not this host's
    form.directAddress = world.address();
    form.displayName = "Intruder";
    form.robotPath = robotPath.toStdString();
    REQUIRE(controller.join(form).ok());

    REQUIRE(tickUntil(controller, [&] { return controller.sessionState().phase == cl::LinkPhase::Failed; }, 15s));
    const cl::Presentation& p = controller.presentation();
    CHECK(p.screen == cl::Screen::Join);
    CHECK(p.banner == cl::Banner::LinkLost);
    CHECK_FALSE(p.detail.empty());
    CHECK(controller.view() == nullptr);
    CHECK(world.hosting->status()->joinsAccepted == 0);
}

TEST_CASE("client e2e: a robot the host refuses comes back to the form with the host's reason") {
    HostWorld world;
    QTemporaryDir dir;
    // The host runs the 2026 game; a robot that names another game is refused as WrongPlugin.
    sdk::RobotConfig other = match_test::swerveSpec("Wrong Game");
    other.gamePluginId = "game_1999";
    const QString path = dir.filePath(QStringLiteral("wrong.json"));
    QFile file(path);
    REQUIRE(file.open(QIODevice::WriteOnly));
    file.write(QByteArray::fromStdString(match_test::specJson(other)));
    file.close();

    auto services = cl::ClientServices::create().value();
    app::MemoryKeyValueStore store;
    FakeInput input;
    cl::ClientController controller(*services, store, &input);
    cl::JoinFormInput form;
    form.invite = world.invite;
    form.directAddress = world.address();
    form.displayName = "Wrong Game";
    form.robotPath = path.toStdString();
    const cl::JoinValidation v = controller.join(form);
    REQUIRE(v.ok());

    REQUIRE(tickUntil(controller, [&] { return controller.presentation().banner == cl::Banner::Refused; }, 15s));
    CHECK(controller.presentation().screen == cl::Screen::Join);
    REQUIRE(controller.presentation().rejectReason.has_value());
    CHECK(*controller.presentation().rejectReason == wire::JoinRejectReason::WrongPlugin);
    CHECK_FALSE(controller.presentation().detail.empty());
}

TEST_CASE("client e2e: when the host closes the room the client says so and does not offer a reconnect") {
    auto world = std::make_unique<HostWorld>();
    QTemporaryDir dir;
    const QString robotPath = writeRobot(dir, false);
    auto services = cl::ClientServices::create().value();
    app::MemoryKeyValueStore store;
    FakeInput input;
    cl::ClientController controller(*services, store, &input);

    cl::JoinFormInput form;
    form.invite = world->invite;
    form.directAddress = world->address();
    form.displayName = "Guest";
    form.robotPath = robotPath.toStdString();
    REQUIRE(controller.join(form).ok());
    REQUIRE(tickUntil(controller, [&] { return isLive(controller); }));

    world->loop->stop();
    world->hosting->shutdown();
    REQUIRE(tickUntil(controller, [&] { return controller.presentation().banner == cl::Banner::ServerClosing; }, 10s));
    CHECK(controller.presentation().screen == cl::Screen::Drive);
    CHECK_FALSE(controller.presentation().canReconnect);
    CHECK_FALSE(controller.presentation().controlsEnabled);
    controller.leave();
    REQUIRE(tickUntil(controller, [&] { return controller.sessionState().phase == cl::LinkPhase::Idle; }, 5s));
}
