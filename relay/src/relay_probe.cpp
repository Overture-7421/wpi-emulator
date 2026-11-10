#include "wpiemu/relay/relay_probe.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <future>
#include <iomanip>
#include <memory>
#include <mutex>
#include <ostream>
#include <sstream>
#include <thread>

#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/net/invite.h"
#include "wpiemu/net/relay_transport.h"
#include "wpiemu/relay/relay_log.h"
#include "wpiemu/relay/relay_selfcheck.h"
#include "wpiemu/sdk/match/match_control.h"

namespace wpiemu::relay {

namespace rl = sdk::match::relay;
namespace proto = sdk::proto;
using Clock = std::chrono::steady_clock;
using Milliseconds = std::chrono::milliseconds;
using Bytes = std::vector<std::byte>;

namespace {

constexpr std::int64_t kMinTimeoutMs = 100;
constexpr std::int64_t kMaxTimeoutMs = 10 * 60 * 1000;
// A name lookup that never returns is the one thing the deadlines cannot bound; the watchdog allows this much beyond it.
constexpr Milliseconds kWatchdogGrace{3000};

// ---- command line -------------------------------------------------------------------------------------------------------

sdk::Error usageError(const std::string& message) { return sdk::Error(sdk::ErrorCode::InvalidArgument, message); }

bool parseNumber(const std::string& text, std::int64_t& out) {
    if (text.empty()) return false;
    const char* first = text.data();
    const char* last = text.data() + text.size();
    const auto [end, ec] = std::from_chars(first, last, out);
    return ec == std::errc() && end == last;
}

// HOST[:PORT], [IPV6]:PORT or a bare IPv6 literal.
sdk::Status parseRelayAddress(const std::string& text, ProbeOptions& options) {
    std::string host = text;
    std::string port;
    if (!text.empty() && text.front() == '[') {
        const std::size_t close = text.find(']');
        if (close == std::string::npos) return usageError("--relay: missing ']' in '" + text + "'");
        host = text.substr(1, close - 1);
        const std::string rest = text.substr(close + 1);
        if (!rest.empty()) {
            if (rest.front() != ':') return usageError("--relay: expected ':PORT' after ']' in '" + text + "'");
            port = rest.substr(1);
            if (port.empty()) return usageError("--relay: empty port in '" + text + "'");
        }
    } else if (std::count(text.begin(), text.end(), ':') == 1) {
        const std::size_t colon = text.find(':');
        host = text.substr(0, colon);
        port = text.substr(colon + 1);
        if (port.empty()) return usageError("--relay: empty port in '" + text + "'");
    }
    if (host.empty()) return usageError("--relay: no host in '" + text + "'");
    if (host.find_first_of(" /\\\t") != std::string::npos) return usageError("--relay: '" + text + "' is not HOST[:PORT]");
    options.relayHost = host;
    if (!port.empty()) {
        std::int64_t value = 0;
        if (!parseNumber(port, value) || value < 1 || value > 65535) return usageError("--relay: port must be 1..65535, got '" + port + "'");
        options.relayPort = static_cast<std::uint16_t>(value);
    }
    return sdk::Status();
}

std::string hostPortText(const std::string& host, std::uint16_t port) {
    const bool v6 = host.find(':') != std::string::npos;
    return (v6 ? "[" + host + "]" : host) + ":" + std::to_string(port);
}

// ---- measurements -------------------------------------------------------------------------------------------------------

double millisBetween(Clock::time_point from, Clock::time_point to) {
    return std::chrono::duration<double, std::milli>(to - from).count();
}

RttSummary summarize(std::vector<double> samples) {
    RttSummary summary;
    if (samples.empty()) return summary;
    std::sort(samples.begin(), samples.end());
    summary.samples = samples.size();
    summary.minMs = samples.front();
    summary.maxMs = samples.back();
    double sum = 0;
    for (const double s : samples) sum += s;
    summary.avgMs = sum / static_cast<double>(samples.size());
    summary.p95Ms = percentileOfSorted(samples, 0.95);
    return summary;
}

std::string fixed(double value, int digits = 1) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::fixed << std::setprecision(digits) << value;
    return out.str();
}

std::string rttText(const RttSummary& r) {
    return "rtt ms min " + fixed(r.minMs) + " avg " + fixed(r.avgMs) + " p95 " + fixed(r.p95Ms) + " max " + fixed(r.maxMs);
}

// ---- the run ------------------------------------------------------------------------------------------------------------

class Run {
public:
    Run(const ProbeOptions& options) : options_(options), start_(Clock::now()), deadline_(start_ + options.timeout) {
        result.mode = options.mode;
        result.relay = hostPortText(options.relayHost, options.relayPort);
    }

    Milliseconds left() const {
        const auto remaining = std::chrono::duration_cast<Milliseconds>(deadline_ - Clock::now());
        return remaining.count() > 0 ? remaining : Milliseconds{0};
    }
    bool expired() const { return left().count() <= 0; }
    Clock::time_point deadline() const { return deadline_; }
    Clock::time_point start() const { return start_; }

    // Records a step that began at `since`; the first failed step supplies the probe's error text.
    bool record(const std::string& name, bool ok, Clock::time_point since, const std::string& detail, const std::string& advice = {}) {
        ProbeStep step;
        step.name = name;
        step.ok = ok;
        step.ms = static_cast<std::int64_t>(std::llround(millisBetween(since, Clock::now())));
        step.detail = detail;
        result.steps.push_back(std::move(step));
        if (!ok && result.error.empty()) result.error = name + ": " + (advice.empty() ? detail : advice);
        return ok;
    }

    ProbeResult result;

private:
    const ProbeOptions& options_;
    Clock::time_point start_;
    Clock::time_point deadline_;
};

// The text a failed liveness check turns into: what went wrong and what to look at.
std::string adviceFor(const SelfCheckResult& check, const ProbeOptions& options, Milliseconds timeout) {
    const std::string where = hostPortText(options.relayHost, options.relayPort);
    switch (check.failure) {
        case SelfCheckFailure::Resolve:
            return "DNS failure: cannot resolve '" + options.relayHost + "' (check the name and this machine's DNS)";
        case SelfCheckFailure::Unreachable:
            return "no UDP response from " + where + " within " + std::to_string(timeout.count()) +
                   " ms (UDP may be blocked on this network, a firewall may drop it, or nothing listens at that address and port)";
        case SelfCheckFailure::Connect: return "cannot connect to " + where + ": " + check.detail;
        case SelfCheckFailure::NoAnswer:
            return "protocol mismatch: connected to " + where + " over ENet but JoinRoom got no answer within " +
                   std::to_string(timeout.count()) + " ms (not a wpiemu relay, or an incompatible version)";
        case SelfCheckFailure::HungUp:
            return "protocol mismatch: " + where + " closed the connection after JoinRoom (not a wpiemu relay, or an incompatible version)";
        case SelfCheckFailure::Protocol: return "protocol mismatch: " + check.detail;
        case SelfCheckFailure::None: break;
    }
    return check.detail;
}

// Connect and ask for the reserved health-ping room. `strict` also requires the answer to be NoSuchRoom.
bool reachStep(Run& run, const ProbeOptions& options, bool strict) {
    const auto since = Clock::now();
    const Milliseconds timeout = std::max(run.left(), Milliseconds{1});
    const SelfCheckResult check = runSelfCheck(options.relayHost, options.relayPort, timeout);
    if (!check.resolvedAddress.empty()) run.result.resolvedAddress = check.resolvedAddress;
    const std::string name = options.mode == ProbeMode::SelfCheck ? "selfcheck" : "reach";
    if (!check.ok) return run.record(name, false, since, check.detail, adviceFor(check, options, timeout));
    const rl::JoinResult answer = check.answer.value_or(rl::JoinResult{});
    if (strict && answer.reason != rl::JoinRoomFailure::NoSuchRoom) {
        if (answer.reason == rl::JoinRoomFailure::BadVersion)
            return run.record(name, false, since, "the relay refused our protocol version",
                              "protocol mismatch: the relay does not speak relay protocol " + std::to_string(rl::kRelayProtocolVersion));
        if (answer.reason == rl::JoinRoomFailure::LockedOut)
            return run.record(name, false, since, "this source address is locked out",
                              "the relay locked this network address out after too many failed joins; wait or use another network");
        return run.record(name, false, since, "unexpected answer (reason " + std::to_string(static_cast<int>(answer.reason)) + ")",
                          "the relay answered a join for the health-ping room with an unexpected reason");
    }
    return run.record(name, true, since,
                      options.mode == ProbeMode::SelfCheck ? "the relay answered" : "the relay answered NoSuchRoom for the health-ping room" +
                                                                                       (run.result.resolvedAddress.empty() ? std::string()
                                                                                                                         : " (" + run.result.resolvedAddress + ")"));
}

sdk::match::MatchControl pingMessage(sdk::match::MatchControlKind kind, std::uint32_t seq, std::int64_t micros) {
    sdk::match::MatchControl message;
    message.kind = kind;
    message.pingSeq = seq;
    message.clientTimeUs = micros;
    message.detail = "wpiemu-probe";
    return message;
}

// The next MatchControl of `kind` and sequence `seq` on `channel` before `deadline`; anything else is skipped.
std::optional<sdk::match::MatchControl> awaitPing(proto::MessageChannel& channel, sdk::match::MatchControlKind kind,
                                                  std::uint32_t seq, Clock::time_point deadline) {
    while (Clock::now() < deadline) {
        proto::Message message;
        const sdk::Result<bool> got = channel.receive(message, std::min(Milliseconds{100}, proto::timeLeft(deadline)));
        if (!got) return std::nullopt;
        if (!got.value()) continue;
        if (const auto* control = std::get_if<sdk::match::MatchControl>(&message))
            if (control->kind == kind && control->pingSeq == seq) return *control;
    }
    return std::nullopt;
}

// One request answered by the host, both legs over `delivery`; the round-trip time in milliseconds, or nullopt.
std::optional<double> pingEcho(proto::MessageChannel& client, proto::MessageChannel& host, proto::Delivery delivery, std::uint32_t seq,
                               Clock::time_point origin, Clock::time_point deadline) {
    using sdk::match::MatchControlKind;
    const auto sent = Clock::now();
    const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(sent - origin).count();
    if (!client.send(pingMessage(MatchControlKind::PingRequest, seq, micros), delivery)) return std::nullopt;
    const auto request = awaitPing(host, MatchControlKind::PingRequest, seq, deadline);
    if (!request) return std::nullopt;
    if (!host.send(pingMessage(MatchControlKind::PingReply, seq, request->clientTimeUs), delivery)) return std::nullopt;
    const auto reply = awaitPing(client, MatchControlKind::PingReply, seq, deadline);
    if (!reply || reply->clientTimeUs != micros) return std::nullopt;
    return millisBetween(sent, Clock::now());
}

Bytes patternBytes(std::size_t size) {
    Bytes out(size);
    std::uint64_t state = 0x9E3779B97F4A7C15ULL;
    for (std::size_t i = 0; i < size; ++i) {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        out[i] = static_cast<std::byte>(state & 0xFF);
    }
    return out;
}

// The frame equal to `expected` on `transport` before `deadline`; others (late datagrams) are skipped.
bool awaitFrame(proto::ITransport& transport, const Bytes& expected, Clock::time_point deadline) {
    Bytes frame;
    while (Clock::now() < deadline) {
        const sdk::Result<bool> got = transport.receive(frame, std::min(Milliseconds{100}, proto::timeLeft(deadline)));
        if (!got) return false;
        if (got.value() && frame == expected) return true;
    }
    return false;
}

std::string describeError(const sdk::Error& error) { return error.message.empty() ? "unknown error" : error.message; }

// The player path against the relay. Everything it opens closes on every exit through the destructors, and the room is
// left explicitly (a "leave" step) so it does not linger until its TTL.
void fullSteps(Run& run, const ProbeOptions& options) {
    // ---- invite
    auto since = Clock::now();
    std::optional<net::InviteSecret> secret;
    if (options.invite.empty()) {
        auto generated = net::generateInvite();
        if (!generated) {
            run.record("invite", false, since, describeError(generated.error()));
            return;
        }
        secret.emplace(std::move(generated).value());
    } else {
        auto parsed = net::parseInvite(options.invite);
        if (!parsed) {
            run.record("invite", false, since, "bad --invite: " + describeError(parsed.error()));
            return;
        }
        secret.emplace(std::move(parsed).value());
    }
    run.result.room = net::roomId(*secret);
    if (options.printInvite) run.result.invite = net::inviteText(*secret);
    run.record("invite", true, since, std::string(options.invite.empty() ? "fresh invite" : "given invite") + ", room " + run.result.room);

    // ---- register
    since = Clock::now();
    net::RelayHostOptions hostOptions = net::relayHostOptionsFor(options.relayHost, options.relayPort, *secret);
    hostOptions.autoReregister = false;
    const Milliseconds half = std::max(run.left() / 2, Milliseconds{kMinTimeoutMs});
    hostOptions.connectTimeout = half;
    hostOptions.registerTimeout = half;
    auto host = std::make_unique<net::RelayHostTransport>(hostOptions);
    if (const sdk::Status listening = host->listen(); !listening) {
        const sdk::Error& error = listening.error();
        std::string advice = describeError(error);
        if (error.code == sdk::ErrorCode::AlreadyExists) advice += " (a room for this invite is already registered: use another invite)";
        else if (error.code == sdk::ErrorCode::OutOfRange) advice += " (the relay has no room left)";
        run.record("register", false, since, describeError(error), advice);
        return;
    }
    run.record("register", true, since, "registered room " + run.result.room);

    // ---- join
    since = Clock::now();
    net::RelayClientTransport joiner;
    auto joined = joiner.connect(host->endpoint(), std::max(run.left(), Milliseconds{1}));
    if (!joined) {
        run.record("join", false, since, describeError(joined.error()));
        return;
    }
    std::unique_ptr<proto::ITransport> clientLink = std::move(joined).value();
    auto accepted = host->accept(std::max(run.left(), Milliseconds{1}));
    if (!accepted || accepted.value() == nullptr) {
        run.record("join", false, since,
                   accepted ? "the relay did not announce the client to the host in time" : describeError(accepted.error()));
        return;
    }
    std::unique_ptr<proto::ITransport> hostLink = std::move(accepted).value();
    run.record("join", true, since, "joined as " + clientLink->describe());

    // ---- encrypted handshake, both halves at once
    since = Clock::now();
    const Milliseconds handshakeBudget = std::max(run.left(), Milliseconds{1});
    auto clientSide = std::async(std::launch::async, [&] {
        return net::SecureTransport::connectAsClient(std::move(clientLink), *secret, handshakeBudget);
    });
    auto hostSide = net::SecureTransport::acceptAsHost(std::move(hostLink), *secret, handshakeBudget);
    auto clientSecure = clientSide.get();
    if (!clientSecure || !hostSide) {
        run.record("handshake", false, since, !clientSecure ? "client: " + describeError(clientSecure.error()) : "host: " + describeError(hostSide.error()));
        return;
    }
    run.record("handshake", true, since, "encrypted session established");

    const auto codec = std::make_shared<ipc::BinaryMessageCodec>();
    proto::MessageChannel client(std::move(clientSecure).value(), codec);
    proto::MessageChannel hostChannel(std::move(hostSide).value(), codec);
    std::vector<double> samples;

    // ---- reliable echoes
    if (options.count > 0) {
        since = Clock::now();
        std::vector<double> own;
        bool failed = false;
        for (std::size_t i = 0; i < options.count && !failed; ++i) {
            const auto rtt = pingEcho(client, hostChannel, proto::Delivery::Reliable, static_cast<std::uint32_t>(i + 1), run.start(), run.deadline());
            if (rtt) own.push_back(*rtt);
            else failed = true;
        }
        if (failed) {
            run.record("reliable-echo", false, since, "echo " + std::to_string(own.size() + 1) + " of " + std::to_string(options.count) + " got no reply before the deadline");
        } else {
            run.record("reliable-echo", true, since, std::to_string(own.size()) + " echoes, " + rttText(summarize(own)));
        }
        samples.insert(samples.end(), own.begin(), own.end());
        if (failed) {
            run.result.rtt = summarize(samples);
            return;
        }

        // ---- unreliable echoes: lost ones are counted, and the phase has its own share of the time
        since = Clock::now();
        std::vector<double> lossy;
        const auto phaseEnd = since + run.left() / 2;
        for (std::size_t i = 0; i < options.count && Clock::now() < phaseEnd; ++i) {
            const auto limit = std::min(phaseEnd, Clock::now() + Milliseconds{1000});
            const auto rtt = pingEcho(client, hostChannel, proto::Delivery::Unreliable, static_cast<std::uint32_t>(10'000 + i), run.start(), limit);
            if (rtt) lossy.push_back(*rtt);
        }
        const std::size_t lost = options.count - lossy.size();
        run.result.unreliableLossPermille = static_cast<std::uint32_t>(lost * 1000 / options.count);
        const bool any = !lossy.empty();
        run.record("unreliable-echo", any, since,
                   std::to_string(lossy.size()) + " of " + std::to_string(options.count) + " datagram echoes came back (loss " +
                       std::to_string(run.result.unreliableLossPermille) + " permille)" + (any ? ", " + rttText(summarize(lossy)) : std::string()),
                   "no unreliable datagram came back: UDP game traffic may be filtered on this path");
        samples.insert(samples.end(), lossy.begin(), lossy.end());
        run.result.rtt = summarize(samples);
        if (!any) return;
    }

    // ---- a large reliable round trip
    if (options.payloadBytes > 0) {
        since = Clock::now();
        const Bytes payload = patternBytes(options.payloadBytes);
        proto::ITransport& clientTransport = client.transport();
        proto::ITransport& hostTransport = hostChannel.transport();
        bool ok = clientTransport.send(payload).ok();
        ok = ok && awaitFrame(hostTransport, payload, run.deadline());
        ok = ok && hostTransport.send(payload).ok();
        ok = ok && awaitFrame(clientTransport, payload, run.deadline());
        const double ms = millisBetween(since, Clock::now());
        if (ok) {
            run.result.reliableBytes = options.payloadBytes;
            run.record("payload", true, since, std::to_string(options.payloadBytes) + " bytes echoed intact in " + fixed(ms) + " ms");
        } else {
            run.record("payload", false, since, std::to_string(options.payloadBytes) + " bytes were not echoed intact before the deadline");
        }
    }

    // ---- leave: both ends disconnect now, the relay frees the room without waiting for its TTL
    since = Clock::now();
    client.close();
    hostChannel.close();
    host->close();
    run.record("leave", true, since, "closed the client and the host (the host sent CloseRoom)");
}

}  // namespace

const char* probeModeName(ProbeMode mode) {
    switch (mode) {
        case ProbeMode::Full: return "full";
        case ProbeMode::Reach: return "reach";
        case ProbeMode::SelfCheck: return "selfcheck";
    }
    return "full";
}

double percentileOfSorted(const std::vector<double>& sorted, double fraction) {
    if (sorted.empty()) return 0;
    const double rank = std::ceil(fraction * static_cast<double>(sorted.size()));
    const std::size_t index = static_cast<std::size_t>(std::clamp(rank, 1.0, static_cast<double>(sorted.size()))) - 1;
    return sorted[index];
}

std::string probeUsage() {
    std::ostringstream out;
    out << "usage: wpiemu_relay_probe --relay HOST[:PORT] [options]\n"
           "Checks a running relay from the outside. Exit code 0: the probe passed, 1: it failed, 2: usage error.\n"
           "\n"
           "  --relay HOST[:PORT]    the relay (default port " << kDefaultRelayPort << "; [IPV6]:PORT for IPv6); or set "
        << kProbeRelayEnvironment << "\n"
           "  --mode full|reach|selfcheck\n"
           "                         full (default): register a room, join it, run the encrypted handshake, echo --count\n"
           "                         messages on both delivery classes and a --payload-bytes reliable round trip, then leave.\n"
           "                         reach: UDP reachability and a relay that answers; creates nothing.\n"
           "                         selfcheck: what `wpiemu_relay --selfcheck` does.\n"
           "  --timeout-ms N         the deadline of the whole probe, " << kMinTimeoutMs << ".." << kMaxTimeoutMs << " (default "
        << kProbeDefaultTimeout.count() << ")\n"
           "  --count N              echoes per delivery class, 0.." << kProbeMaxCount << " (default " << kProbeDefaultCount << ")\n"
           "  --payload-bytes N      the reliable round trip, 0.." << kProbeMaxPayloadBytes << ", 0 skips it (default "
        << kProbeDefaultPayloadBytes << ")\n"
           "  --invite XXXXX-XXXXX-XXXXX-XXXXX\n"
           "                         full mode: use this invite instead of a fresh one\n"
           "  --print-invite         full mode: print the invite secret (never printed otherwise; only the room id is)\n"
           "  --json                 one line of JSON instead of text\n"
           "  --version, --help\n";
    return out.str();
}

sdk::Result<ProbeInvocation> parseProbeArgs(const std::vector<std::string>& args, const EnvLookup& env) {
    ProbeInvocation invocation;
    ProbeOptions& options = invocation.options;
    std::optional<std::string> relayText;

    for (std::size_t i = 0; i < args.size(); ++i) {
        std::string flag = args[i];
        std::optional<std::string> inlineValue;
        if (flag.rfind("--", 0) == 0) {
            if (const std::size_t eq = flag.find('='); eq != std::string::npos) {
                inlineValue = flag.substr(eq + 1);
                flag = flag.substr(0, eq);
            }
        }
        const auto value = [&]() -> sdk::Result<std::string> {
            if (inlineValue) return *inlineValue;
            if (i + 1 >= args.size()) return usageError(flag + ": a value is required");
            return args[++i];
        };
        const auto number = [&](std::int64_t low, std::int64_t high, std::int64_t& out) -> sdk::Status {
            auto text = value();
            if (!text) return text.error();
            if (!parseNumber(text.value(), out) || out < low || out > high)
                return usageError(flag + ": expected a whole number " + std::to_string(low) + ".." + std::to_string(high) + ", got '" + text.value() + "'");
            return sdk::Status();
        };

        if (flag == "--help" || flag == "-h") {
            invocation.command = ProbeCommand::Help;
            return invocation;
        }
        if (flag == "--version") {
            invocation.command = ProbeCommand::Version;
            return invocation;
        }
        if (flag == "--json" && !inlineValue) {
            options.json = true;
        } else if (flag == "--print-invite" && !inlineValue) {
            options.printInvite = true;
        } else if (flag == "--relay") {
            auto text = value();
            if (!text) return text.error();
            relayText = text.value();
        } else if (flag == "--mode") {
            auto text = value();
            if (!text) return text.error();
            if (text.value() == "full") options.mode = ProbeMode::Full;
            else if (text.value() == "reach") options.mode = ProbeMode::Reach;
            else if (text.value() == "selfcheck") options.mode = ProbeMode::SelfCheck;
            else return usageError("--mode: expected full, reach or selfcheck, got '" + text.value() + "'");
        } else if (flag == "--timeout-ms") {
            std::int64_t n = 0;
            if (const sdk::Status s = number(kMinTimeoutMs, kMaxTimeoutMs, n); !s) return s.error();
            options.timeout = Milliseconds{n};
        } else if (flag == "--count") {
            std::int64_t n = 0;
            if (const sdk::Status s = number(0, static_cast<std::int64_t>(kProbeMaxCount), n); !s) return s.error();
            options.count = static_cast<std::size_t>(n);
        } else if (flag == "--payload-bytes") {
            std::int64_t n = 0;
            if (const sdk::Status s = number(0, static_cast<std::int64_t>(kProbeMaxPayloadBytes), n); !s) return s.error();
            options.payloadBytes = static_cast<std::size_t>(n);
        } else if (flag == "--invite") {
            auto text = value();
            if (!text) return text.error();
            if (auto parsed = net::parseInvite(text.value()); !parsed) return usageError("--invite: " + describeError(parsed.error()));
            options.invite = text.value();
        } else {
            return usageError("unknown argument '" + args[i] + "' (try --help)");
        }
    }

    if (!relayText && env) relayText = env(kProbeRelayEnvironment);
    if (!relayText || relayText->empty())
        return usageError(std::string("no relay given: use --relay HOST[:PORT] or set ") + kProbeRelayEnvironment);
    if (const sdk::Status parsed = parseRelayAddress(*relayText, options); !parsed) return parsed.error();
    return invocation;
}

ProbeResult runProbe(const ProbeOptions& options) {
    Run run(options);
    try {
        const bool strict = options.mode != ProbeMode::SelfCheck;
        if (reachStep(run, options, strict) && options.mode == ProbeMode::Full) fullSteps(run, options);
    } catch (const std::exception& e) {
        run.record("probe", false, run.start(), std::string("unexpected failure: ") + e.what());
    }
    run.result.ok = !run.result.steps.empty() && std::all_of(run.result.steps.begin(), run.result.steps.end(), [](const ProbeStep& s) { return s.ok; });
    if (run.result.ok) run.result.error.clear();
    return run.result;
}

std::string formatProbeText(const ProbeResult& r) {
    std::ostringstream out;
    out << "wpiemu_relay_probe: relay " << r.relay;
    if (!r.resolvedAddress.empty()) out << " (" << r.resolvedAddress << ")";
    out << ", mode " << probeModeName(r.mode) << "\n";
    for (const ProbeStep& step : r.steps) out << "  " << (step.ok ? "ok  " : "FAIL") << "  " << std::left << std::setw(16) << step.name << std::right << std::setw(6) << step.ms << " ms  " << step.detail << "\n";
    if (r.mode == ProbeMode::Full && r.rtt.samples > 0)
        out << "  " << rttText(r.rtt) << " over " << r.rtt.samples << " echoes, unreliable loss " << r.unreliableLossPermille << " permille, reliable payload "
            << r.reliableBytes << " bytes\n";
    if (!r.invite.empty()) out << "  invite " << r.invite << "\n";
    if (r.ok) out << "RESULT: ok\n";
    else out << "RESULT: FAILED: " << r.error << "\n";
    return out.str();
}

std::string formatProbeJson(const ProbeResult& r) {
    std::string out = "{\"ok\":";
    out += r.ok ? "true" : "false";
    out += ",\"mode\":";
    appendJsonString(out, probeModeName(r.mode));
    out += ",\"relay\":";
    appendJsonString(out, r.relay);
    if (!r.resolvedAddress.empty()) {
        out += ",\"resolvedAddress\":";
        appendJsonString(out, r.resolvedAddress);
    }
    out += ",\"rttMs\":{\"min\":" + fixed(r.rtt.minMs, 3) + ",\"avg\":" + fixed(r.rtt.avgMs, 3) + ",\"p95\":" + fixed(r.rtt.p95Ms, 3) +
           ",\"max\":" + fixed(r.rtt.maxMs, 3) + "}";
    out += ",\"unreliableLossPermille\":" + std::to_string(r.unreliableLossPermille);
    out += ",\"reliableBytes\":" + std::to_string(r.reliableBytes);
    out += ",\"steps\":[";
    for (std::size_t i = 0; i < r.steps.size(); ++i) {
        if (i > 0) out += ",";
        out += "{\"name\":";
        appendJsonString(out, r.steps[i].name);
        out += r.steps[i].ok ? ",\"ok\":true" : ",\"ok\":false";
        out += ",\"ms\":" + std::to_string(r.steps[i].ms) + ",\"detail\":";
        appendJsonString(out, r.steps[i].detail);
        out += "}";
    }
    out += "]";
    if (!r.room.empty()) {
        out += ",\"room\":";
        appendJsonString(out, r.room);
    }
    if (!r.invite.empty()) {
        out += ",\"invite\":";
        appendJsonString(out, r.invite);
    }
    if (!r.error.empty()) {
        out += ",\"error\":";
        appendJsonString(out, r.error);
    }
    out += "}";
    return out;
}

namespace {

struct WatchdogState {
    std::mutex mutex;
    std::condition_variable wake;
    bool done = false;
};

void startWatchdog(const std::shared_ptr<WatchdogState>& state, const ProbeOptions& options, std::ostream& out) {
    std::thread([state, options, &out] {
        std::unique_lock lock(state->mutex);
        if (state->wake.wait_for(lock, options.timeout + kWatchdogGrace, [&] { return state->done; })) return;
        ProbeResult result;
        result.mode = options.mode;
        result.relay = hostPortText(options.relayHost, options.relayPort);
        result.error = "the probe did not finish within " + std::to_string((options.timeout + kWatchdogGrace).count()) +
                       " ms (a name lookup or a blocked network call that ignores the deadline)";
        out << (options.json ? formatProbeJson(result) + "\n" : formatProbeText(result)) << std::flush;
        std::_Exit(1);
    }).detach();
}

}  // namespace

int runProbeCli(const std::vector<std::string>& args, const EnvLookup& env, std::ostream& out, std::ostream& err, bool watchdog) {
    const sdk::Result<ProbeInvocation> parsed = parseProbeArgs(args, env);
    if (!parsed) {
        err << "wpiemu_relay_probe: " << parsed.error().message << "\n";
        return 2;
    }
    const ProbeInvocation& invocation = parsed.value();
    if (invocation.command == ProbeCommand::Help) {
        out << probeUsage();
        return 0;
    }
    if (invocation.command == ProbeCommand::Version) {
        out << "wpiemu_relay_probe " << kRelayVersion << " (relay protocol " << rl::kRelayProtocolVersion << ")\n";
        return 0;
    }

    auto state = std::make_shared<WatchdogState>();
    if (watchdog) startWatchdog(state, invocation.options, out);
    const ProbeResult result = runProbe(invocation.options);
    {
        std::lock_guard lock(state->mutex);
        state->done = true;
    }
    state->wake.notify_all();

    if (invocation.options.json) {
        out << formatProbeJson(result) << "\n";
    } else {
        out << formatProbeText(result);
        if (!result.ok) err << "wpiemu_relay_probe: " << result.error << "\n";
    }
    return result.ok ? 0 : 1;
}

}  // namespace wpiemu::relay
