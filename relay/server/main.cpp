// wpiemu_relay: the practice-match relay (UDP, ENet). See relay/include/wpiemu/relay/relay_options.h for the options.

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "wpiemu/net/enet_transport.h"
#include "wpiemu/relay/relay_log.h"
#include "wpiemu/relay/relay_options.h"
#include "wpiemu/relay/relay_selfcheck.h"
#include "wpiemu/relay/relay_server.h"
#include "wpiemu/sdk/core/clock.h"
#include "wpiemu/sdk/match/relay_messages.h"

namespace {

using namespace wpiemu;

std::atomic<relay::RelayServer*> gServer{nullptr};

void requestStop() {
    if (relay::RelayServer* server = gServer.load()) server->stop();
}

#ifdef _WIN32
BOOL WINAPI onConsoleControl(DWORD type) {
    switch (type) {
        case CTRL_C_EVENT:
        case CTRL_BREAK_EVENT:
        case CTRL_CLOSE_EVENT:
        case CTRL_SHUTDOWN_EVENT: requestStop(); return TRUE;
        default: return FALSE;
    }
}
void installSignalHandlers() { SetConsoleCtrlHandler(onConsoleControl, TRUE); }
#else
void onSignal(int) { requestStop(); }
void installSignalHandlers() {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
}
#endif

int runRelay(const relay::RelayOptions& options) {
    auto log = std::make_shared<relay::RelayLog>(options.logFormat);

    auto server = std::make_unique<net::EnetServerTransport>(relay::transportOptionsFor(options));

    const relay::RoomTableConfig& t = options.server.table;
    log->log(relay::LogLevel::Info, "starting",
             {{"version", relay::kRelayVersion},
              {"bind", options.bindHost},
              {"port", options.port},
              {"maxRooms", t.maxRooms},
              {"maxPeersPerRoom", t.maxPeersPerRoom},
              {"roomTtlMs", t.roomTtlWithoutHostMs},
              {"peerBytesPerSec", t.peerBytesPerSecond},
              {"roomBytesPerSec", t.roomBytesPerSecond},
              {"lockoutFailures", t.lockoutMaxFailures},
              {"lockoutWindowMs", t.lockoutWindowMs},
              {"lockoutDurationMs", t.lockoutDurationMs},
              {"maxConnections", options.maxConnections}});

    relay::RelayServer relayServer(std::move(server), options.server, std::make_shared<sdk::SteadyWallClock>(), log);
    gServer.store(&relayServer);
    installSignalHandlers();
    const sdk::Status status = relayServer.run();
    gServer.store(nullptr);
    if (!status) {
        log->log(relay::LogLevel::Error, "failed", {{"error", status.error().message}});
        return 1;
    }
    log->log(relay::LogLevel::Info, "stopped");
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv + 1, argv + argc);
    const sdk::Result<relay::RelayOptions> parsed = relay::parseRelayOptions(args, relay::processEnvironment);
    if (!parsed) {
        std::cerr << "wpiemu_relay: " << parsed.error().message << "\n";
        return 2;
    }
    const relay::RelayOptions& options = parsed.value();

    switch (options.command) {
        case relay::RelayCommand::Help: std::cout << relay::relayUsage(); return 0;
        case relay::RelayCommand::Version:
            std::cout << "wpiemu_relay " << relay::kRelayVersion << " (relay protocol "
                      << sdk::match::relay::kRelayProtocolVersion << ")\n";
            return 0;
        case relay::RelayCommand::SelfCheck: {
            const relay::SelfCheckResult result = relay::runSelfCheck(options.selfCheckHost, options.port, options.selfCheckTimeout);
            if (!result.ok) std::cerr << "wpiemu_relay --selfcheck: " << result.detail << "\n";
            return result.ok ? 0 : 1;
        }
        case relay::RelayCommand::Run: return runRelay(options);
    }
    return 2;
}
