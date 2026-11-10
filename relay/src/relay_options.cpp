#include "wpiemu/relay/relay_options.h"

#include <algorithm>
#include <charconv>
#include <sstream>

namespace wpiemu::relay {

namespace rl = sdk::match::relay;

namespace {

using Apply = std::function<sdk::Status(RelayOptions&, const std::string& name, const std::string& value)>;

struct Setting {
    const char* flag;  // "--port"
    const char* env;   // "WPIEMU_RELAY_PORT"; null: command line only
    const char* help;
    Apply apply;
};

sdk::Error invalid(const std::string& name, const std::string& value, const std::string& expected) {
    return sdk::makeError(sdk::ErrorCode::InvalidArgument, name + ": invalid value '" + value + "' (" + expected + ")");
}

sdk::Result<std::uint64_t> parseNumber(const std::string& name, const std::string& value, std::uint64_t min, std::uint64_t max) {
    std::uint64_t parsed = 0;
    const char* begin = value.data();
    const char* end = begin + value.size();
    const auto result = std::from_chars(begin, end, parsed);
    const std::string expected = "an integer from " + std::to_string(min) + " to " + std::to_string(max);
    if (value.empty() || result.ec != std::errc{} || result.ptr != end) return invalid(name, value, expected);
    if (parsed < min || parsed > max) return invalid(name, value, expected);
    return parsed;
}

Apply number(std::uint64_t min, std::uint64_t max, std::function<void(RelayOptions&, std::uint64_t)> set) {
    return [min, max, set](RelayOptions& options, const std::string& name, const std::string& value) -> sdk::Status {
        const sdk::Result<std::uint64_t> parsed = parseNumber(name, value, min, max);
        if (!parsed) return parsed.error();
        set(options, parsed.value());
        return sdk::okStatus();
    };
}

const std::vector<Setting>& settings() {
    static const std::vector<Setting> table = {
        {"--port", "WPIEMU_RELAY_PORT", "UDP port to listen on, and the port --selfcheck connects to (default 7777)",
         number(1, 65535, [](RelayOptions& o, std::uint64_t v) { o.port = static_cast<std::uint16_t>(v); })},
        {"--bind", "WPIEMU_RELAY_BIND", "address to listen on: an IPv4 or IPv6 literal or a name (default ::, dual stack)",
         [](RelayOptions& o, const std::string& name, const std::string& value) -> sdk::Status {
             if (value.empty() || value.size() > 253 || value.find_first_of(" \t\r\n") != std::string::npos)
                 return invalid(name, value, "an address or host name");
             o.bindHost = value;
             return sdk::okStatus();
         }},
        {"--max-rooms", "WPIEMU_RELAY_MAX_ROOMS", "rooms at once (default 100)",
         number(1, kMaxConfigurableRooms, [](RelayOptions& o, std::uint64_t v) { o.server.table.maxRooms = static_cast<std::size_t>(v); })},
        {"--max-peers", "WPIEMU_RELAY_MAX_PEERS", "clients per room (default 6)",
         number(1, rl::kRelayMaxPeersPerRoom, [](RelayOptions& o, std::uint64_t v) { o.server.table.maxPeersPerRoom = static_cast<std::size_t>(v); })},
        {"--room-ttl-ms", "WPIEMU_RELAY_ROOM_TTL_MS", "how long a room survives without its host (default 30000)",
         number(1000, 86'400'000, [](RelayOptions& o, std::uint64_t v) { o.server.table.roomTtlWithoutHostMs = static_cast<std::uint32_t>(v); })},
        {"--peer-bytes-per-sec", "WPIEMU_RELAY_PEER_BYTES_PER_SEC", "token bucket rate per client, bytes per second (default 65536)",
         number(1024, 100'000'000, [](RelayOptions& o, std::uint64_t v) { o.server.table.peerBytesPerSecond = static_cast<std::uint32_t>(v); })},
        {"--room-bytes-per-sec", "WPIEMU_RELAY_ROOM_BYTES_PER_SEC", "forwarded bytes per second per room; 0 derives it from the peer rate (default 0)",
         number(0, 1'000'000'000, [](RelayOptions& o, std::uint64_t v) { o.server.table.roomBytesPerSecond = static_cast<std::uint32_t>(v); })},
        {"--lockout-failures", "WPIEMU_RELAY_LOCKOUT_FAILURES", "failed joins or registrations from one source before the lockout; 0 disables it (default 10)",
         number(0, 10'000, [](RelayOptions& o, std::uint64_t v) { o.server.table.lockoutMaxFailures = static_cast<std::uint32_t>(v); })},
        {"--lockout-window-s", "WPIEMU_RELAY_LOCKOUT_WINDOW_S", "window in which the failures count, seconds (default 60)",
         number(1, 86'400, [](RelayOptions& o, std::uint64_t v) { o.server.table.lockoutWindowMs = static_cast<std::uint32_t>(v * 1000); })},
        {"--lockout-duration-s", "WPIEMU_RELAY_LOCKOUT_DURATION_S", "how long a locked-out source is refused, seconds (default 300)",
         number(1, 86'400, [](RelayOptions& o, std::uint64_t v) { o.server.table.lockoutDurationMs = static_cast<std::uint32_t>(v * 1000); })},
        {"--stats-interval-s", "WPIEMU_RELAY_STATS_INTERVAL_S", "period of the stats log line, seconds; 0 disables it (default 60)",
         number(0, 86'400, [](RelayOptions& o, std::uint64_t v) { o.server.statsInterval = std::chrono::seconds(static_cast<std::int64_t>(v)); })},
        {"--log", "WPIEMU_RELAY_LOG", "log format: json or text (default json)",
         [](RelayOptions& o, const std::string& name, const std::string& value) -> sdk::Status {
             if (value == "json") o.logFormat = LogFormat::Json;
             else if (value == "text") o.logFormat = LogFormat::Text;
             else return invalid(name, value, "json or text");
             return sdk::okStatus();
         }},
        {"--host", nullptr, "--selfcheck only: the relay's host (default 127.0.0.1)",
         [](RelayOptions& o, const std::string& name, const std::string& value) -> sdk::Status {
             if (value.empty() || value.size() > 253 || value.find_first_of(" \t\r\n") != std::string::npos)
                 return invalid(name, value, "an address or host name");
             o.selfCheckHost = value;
             return sdk::okStatus();
         }},
    };
    return table;
}

}  // namespace

std::string relayUsage() {
    std::ostringstream out;
    out << "usage: wpiemu_relay [options]\n"
        << "       wpiemu_relay --selfcheck [--port N] [--host H]\n"
        << "       wpiemu_relay --version | --help\n\n"
        << "A UDP relay that lets a practice-match host and its clients meet without port forwarding. It forwards opaque,\n"
        << "end-to-end encrypted frames and never reads them. Every option below can also be set with the environment\n"
        << "variable in brackets; the environment wins over the command line.\n\n";
    for (const Setting& s : settings()) {
        out << "  " << s.flag << " VALUE";
        if (s.env != nullptr) out << "  [" << s.env << "]";
        out << "\n      " << s.help << "\n";
    }
    out << "  --selfcheck\n      liveness probe: join a random non-existent room on a running relay and expect a refusal within\n"
        << "      2 seconds; exits 0 when the relay answered, 1 otherwise. It creates no room and changes no state.\n"
        << "\nExit codes: 0 ok, 1 runtime failure (or failed selfcheck), 2 invalid configuration or usage.\n";
    return out.str();
}

sdk::Result<RelayOptions> parseRelayOptions(const std::vector<std::string>& args, const EnvLookup& env) {
    RelayOptions options;
    for (std::size_t i = 0; i < args.size(); ++i) {
        std::string arg = args[i];
        if (arg == "--help" || arg == "-h") {
            options.command = RelayCommand::Help;
            continue;
        }
        if (arg == "--version") {
            options.command = RelayCommand::Version;
            continue;
        }
        if (arg == "--selfcheck") {
            options.command = RelayCommand::SelfCheck;
            continue;
        }
        std::string inlineValue;
        bool hasInline = false;
        if (const std::size_t eq = arg.find('='); eq != std::string::npos && arg.rfind("--", 0) == 0) {
            inlineValue = arg.substr(eq + 1);
            arg = arg.substr(0, eq);
            hasInline = true;
        }
        const auto setting = std::find_if(settings().begin(), settings().end(), [&](const Setting& s) { return arg == s.flag; });
        if (setting == settings().end())
            return sdk::makeError(sdk::ErrorCode::InvalidArgument, "unknown option '" + arg + "' (try --help)");
        std::string value = inlineValue;
        if (!hasInline) {
            if (i + 1 >= args.size()) return sdk::makeError(sdk::ErrorCode::InvalidArgument, arg + ": missing value");
            value = args[++i];
        }
        if (const sdk::Status s = setting->apply(options, arg, value); !s) return s.error();
    }

    if (options.command == RelayCommand::Help || options.command == RelayCommand::Version) return options;

    for (const Setting& s : settings()) {
        if (s.env == nullptr || !env) continue;
        const std::optional<std::string> value = env(s.env);
        if (!value || value->empty()) continue;
        if (const sdk::Status status = s.apply(options, s.env, *value); !status) return status.error();
    }

    options.maxConnections = std::min<std::size_t>(
        options.server.table.maxRooms * (options.server.table.maxPeersPerRoom + 1) + 64, 4095);
    return options;
}

net::EnetServerOptions transportOptionsFor(const RelayOptions& options) {
    net::EnetServerOptions transport;
    transport.bindHost = options.bindHost;
    transport.port = options.port;
    transport.maxPeers = options.maxConnections;
    transport.maxReliableFrameBytes = options.server.table.maxReliableBytes + rl::kRelayEnvelopeBytes;
    transport.maxUnreliableFrameBytes = options.server.table.maxDatagramBytes + rl::kRelayEnvelopeBytes;
    return transport;
}

}  // namespace wpiemu::relay
