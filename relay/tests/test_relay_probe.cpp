// The relay probe (relay_probe.h) against a real in-process relay, a relay that misbehaves, a closed port and a plain UDP
// echo. The CLI (runProbeCli) is what the wpiemu_relay_probe executable runs.

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "relay_stack_support.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/net/control_link.h"
#include "wpiemu/net/enet_transport.h"
#include "wpiemu/relay/relay_probe.h"

using namespace relaystack;
namespace rl = wpiemu::sdk::match::relay;

namespace {

// ---- running the CLI ------------------------------------------------------------------------------------------------------

struct CliRun {
    int code = -1;
    std::string out;
    std::string err;
    std::chrono::milliseconds elapsed{0};
};

CliRun cli(const std::vector<std::string>& args, const wr::EnvLookup& env = nullptr) {
    CliRun run;
    std::ostringstream out;
    std::ostringstream err;
    const auto started = std::chrono::steady_clock::now();
    run.code = wr::runProbeCli(args, env, out, err);
    run.elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started);
    run.out = out.str();
    run.err = err.str();
    return run;
}

std::string relayArg(std::uint16_t port) { return "127.0.0.1:" + std::to_string(port); }

// ---- a minimal JSON reader, enough to check the shape of the report --------------------------------------------------------

struct Json {
    enum class Type { Null, Bool, Number, String, Array, Object } type = Type::Null;
    bool boolean = false;
    double number = 0;
    std::string text;
    std::vector<Json> items;
    std::vector<std::string> keys;     // an object's members: keys[i] names values[i] (a pair of an incomplete Json is not allowed)
    std::vector<Json> values;

    const Json* get(const std::string& key) const {
        for (std::size_t i = 0; i < keys.size(); ++i)
            if (keys[i] == key) return &values[i];
        return nullptr;
    }
};

class JsonReader {
public:
    explicit JsonReader(const std::string& text) : text_(text) {}
    std::optional<Json> parseDocument() {
        auto value = parseValue();
        skip();
        if (!value || pos_ != text_.size()) return std::nullopt;
        return value;
    }

private:
    void skip() {
        while (pos_ < text_.size() && (text_[pos_] == ' ' || text_[pos_] == '\n' || text_[pos_] == '\t' || text_[pos_] == '\r')) ++pos_;
    }
    bool take(char c) {
        skip();
        if (pos_ < text_.size() && text_[pos_] == c) {
            ++pos_;
            return true;
        }
        return false;
    }
    std::optional<std::string> parseString() {
        if (!take('"')) return std::nullopt;
        std::string out;
        while (pos_ < text_.size() && text_[pos_] != '"') {
            char c = text_[pos_++];
            if (static_cast<unsigned char>(c) < 0x20) return std::nullopt;
            if (c == '\\') {
                if (pos_ >= text_.size()) return std::nullopt;
                const char e = text_[pos_++];
                switch (e) {
                    case 'n': c = '\n'; break;
                    case 'r': c = '\r'; break;
                    case 't': c = '\t'; break;
                    case '"': c = '"'; break;
                    case '\\': c = '\\'; break;
                    case 'u': pos_ += 4; c = '?'; break;
                    default: return std::nullopt;
                }
            }
            out.push_back(c);
        }
        if (pos_ >= text_.size()) return std::nullopt;
        ++pos_;
        return out;
    }
    std::optional<Json> parseValue() {
        skip();
        if (pos_ >= text_.size()) return std::nullopt;
        Json value;
        const char c = text_[pos_];
        if (c == '{') {
            ++pos_;
            value.type = Json::Type::Object;
            if (take('}')) return value;
            do {
                skip();
                auto key = parseString();
                if (!key || !take(':')) return std::nullopt;
                auto member = parseValue();
                if (!member) return std::nullopt;
                value.keys.push_back(std::move(*key));
                value.values.push_back(std::move(*member));
            } while (take(','));
            if (!take('}')) return std::nullopt;
            return value;
        }
        if (c == '[') {
            ++pos_;
            value.type = Json::Type::Array;
            if (take(']')) return value;
            do {
                auto item = parseValue();
                if (!item) return std::nullopt;
                value.items.push_back(std::move(*item));
            } while (take(','));
            if (!take(']')) return std::nullopt;
            return value;
        }
        if (c == '"') {
            auto s = parseString();
            if (!s) return std::nullopt;
            value.type = Json::Type::String;
            value.text = *s;
            return value;
        }
        for (const auto& [word, boolean] : {std::pair<const char*, bool>{"true", true}, {"false", false}}) {
            if (text_.compare(pos_, std::string(word).size(), word) == 0) {
                pos_ += std::string(word).size();
                value.type = Json::Type::Bool;
                value.boolean = boolean;
                return value;
            }
        }
        std::size_t used = 0;
        try {
            value.number = std::stod(text_.substr(pos_), &used);
        } catch (...) {
            return std::nullopt;
        }
        pos_ += used;
        value.type = Json::Type::Number;
        return value;
    }

    const std::string& text_;
    std::size_t pos_ = 0;
};

Json parseJsonLine(const std::string& output) {
    REQUIRE_MESSAGE(!output.empty(), "the probe printed nothing");
    REQUIRE(output.back() == '\n');
    const std::string line = output.substr(0, output.size() - 1);
    REQUIRE_MESSAGE(line.find('\n') == std::string::npos, "the JSON report must be one line");
    auto parsed = JsonReader(line).parseDocument();
    REQUIRE_MESSAGE(parsed.has_value(), line);
    REQUIRE(parsed->type == Json::Type::Object);
    return std::move(*parsed);
}

// ---- peers that are not a relay ------------------------------------------------------------------------------------------

// An ENet server that reads JoinRoom and answers (or not) as scripted, and ignores everything else.
class ScriptedRelay {
public:
    enum class Behaviour { Silent, Garbage, HangUp, NoSuchRoom, AcceptJoin, RefuseVersion, RoomFull, LockedOut };

    explicit ScriptedRelay(Behaviour behaviour) : behaviour_(behaviour) {
        net::EnetServerOptions options;
        options.bindHost = "127.0.0.1";
        options.port = 0;
        options.maxPeers = 8;
        server_ = std::make_unique<net::EnetServerTransport>(options);
        REQUIRE(server_->listen().ok());
        port_ = net::NetEndpoint::parse(server_->endpoint()).value().port;
        thread_ = std::thread([this] { loop(); });
    }
    ~ScriptedRelay() {
        stop_ = true;
        thread_.join();
    }
    std::uint16_t port() const { return port_; }
    std::size_t controlFrames() const { return controlFrames_.load(); }

private:
    void loop() {
        std::vector<std::unique_ptr<proto::ITransport>> peers;
        const wpiemu::ipc::BinaryMessageCodec codec;
        while (!stop_) {
            auto accepted = server_->accept(10ms);
            if (accepted.ok() && accepted.value() != nullptr) peers.push_back(std::move(accepted).value());
            for (auto& peer : peers) {
                net::IControlLink* control = net::controlLinkOf(*peer);
                Bytes frame;
                const auto got = control->receiveControl(frame, 0ms);
                if (!got || !got.value()) continue;
                ++controlFrames_;
                answer(*peer, *control, codec, frame);
            }
        }
        server_->close();
    }
    void answer(proto::ITransport& peer, net::IControlLink& control, const wpiemu::ipc::BinaryMessageCodec& codec, const Bytes& frame) {
        const auto decoded = codec.decode(frame);
        if (!decoded || std::get_if<rl::JoinRoom>(&decoded.value()) == nullptr) return;  // a registration is never answered
        rl::JoinResult result;
        switch (behaviour_) {
            case Behaviour::Silent: return;
            case Behaviour::Garbage: (void)control.sendControl(Bytes(30, std::byte{0xFF})); return;
            case Behaviour::HangUp: peer.close(); return;
            case Behaviour::NoSuchRoom: result.reason = rl::JoinRoomFailure::NoSuchRoom; break;
            case Behaviour::AcceptJoin: result.ok = true; result.reason = rl::JoinRoomFailure::None; result.peerId = 1; break;
            case Behaviour::RefuseVersion: result.reason = rl::JoinRoomFailure::BadVersion; break;
            case Behaviour::RoomFull: result.reason = rl::JoinRoomFailure::RoomFull; break;
            case Behaviour::LockedOut: result.reason = rl::JoinRoomFailure::LockedOut; break;
        }
        Bytes encoded;
        if (codec.encode(result, encoded).ok()) (void)control.sendControl(encoded);
    }

    Behaviour behaviour_;
    std::unique_ptr<net::EnetServerTransport> server_;
    std::uint16_t port_ = 0;
    std::atomic<bool> stop_{false};
    std::atomic<std::size_t> controlFrames_{0};
    std::thread thread_;
};

// A UDP socket on loopback that sends every datagram it receives straight back.
class UdpEcho {
public:
    UdpEcho() {
#ifdef _WIN32
        WSADATA data{};
        REQUIRE(WSAStartup(MAKEWORD(2, 2), &data) == 0);
#endif
        socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        REQUIRE(socket_ != kInvalid);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
        REQUIRE(::bind(socket_, reinterpret_cast<const sockaddr*>(&address), sizeof address) == 0);
        socklen_t length = sizeof address;
        REQUIRE(::getsockname(socket_, reinterpret_cast<sockaddr*>(&address), &length) == 0);
        port_ = ntohs(address.sin_port);
#ifdef _WIN32
        const DWORD timeoutMs = 20;
        ::setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeoutMs), sizeof timeoutMs);
#else
        timeval timeout{0, 20000};
        ::setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
#endif
        thread_ = std::thread([this] {
            char buffer[2048];
            while (!stop_) {
                sockaddr_in from{};
                socklen_t fromLength = sizeof from;
                const auto n = ::recvfrom(socket_, buffer, sizeof buffer, 0, reinterpret_cast<sockaddr*>(&from), &fromLength);
                if (n > 0) {
                    ++datagrams_;
                    ::sendto(socket_, buffer, static_cast<int>(n), 0, reinterpret_cast<const sockaddr*>(&from), fromLength);
                }
            }
        });
    }
    ~UdpEcho() {
        stop_ = true;
        thread_.join();
#ifdef _WIN32
        ::closesocket(socket_);
        WSACleanup();
#else
        ::close(socket_);
#endif
    }
    std::uint16_t port() const { return port_; }
    std::size_t datagrams() const { return datagrams_.load(); }

private:
#ifdef _WIN32
    using Handle = SOCKET;
    static constexpr Handle kInvalid = INVALID_SOCKET;
#else
    using Handle = int;
    static constexpr Handle kInvalid = -1;
#endif
    Handle socket_ = kInvalid;
    std::uint16_t port_ = 0;
    std::atomic<bool> stop_{false};
    std::atomic<std::size_t> datagrams_{0};
    std::thread thread_;
};

bool contains(const std::string& text, const std::string& needle) { return text.find(needle) != std::string::npos; }

const Json& member(const Json& object, const std::string& key) {
    const Json* found = object.get(key);
    REQUIRE_MESSAGE(found != nullptr, "missing key " << key);
    return *found;
}

}  // namespace

// ---- the command line -----------------------------------------------------------------------------------------------------

TEST_CASE("relay probe: usage errors exit with 2 and say what is wrong") {
    const auto none = [](const std::string&) -> std::optional<std::string> { return std::nullopt; };
    const auto usage = [&](std::vector<std::string> args, const std::string& expected) {
        const CliRun run = cli(args, none);
        CHECK_MESSAGE(run.code == 2, expected);
        CHECK(run.out.empty());
        CHECK_MESSAGE(contains(run.err, expected), run.err);
    };
    usage({}, "no relay given");
    usage({"--relay"}, "a value is required");
    usage({"--relay", ""}, "no relay given");
    usage({"--relay", "host:0"}, "port must be 1..65535");
    usage({"--relay", "host:70000"}, "port must be 1..65535");
    usage({"--relay", "host:abc"}, "port must be 1..65535");
    usage({"--relay", "host:"}, "empty port");
    usage({"--relay", ":7777"}, "no host");
    usage({"--relay", "[::1"}, "missing ']'");
    usage({"--relay", "a/b"}, "not HOST[:PORT]");
    usage({"--relay", "h", "--mode", "soak"}, "--mode: expected full, reach or selfcheck");
    usage({"--relay", "h", "--timeout-ms", "5"}, "--timeout-ms");
    usage({"--relay", "h", "--timeout-ms", "abc"}, "--timeout-ms");
    usage({"--relay", "h", "--count", "-1"}, "--count");
    usage({"--relay", "h", "--count", "1001"}, "--count");
    usage({"--relay", "h", "--payload-bytes", std::to_string(wr::kProbeMaxPayloadBytes + 1)}, "--payload-bytes");
    usage({"--relay", "h", "--invite", "NOT-AN-INVITE"}, "--invite");
    usage({"--relay", "h", "--bogus"}, "unknown argument '--bogus'");
    usage({"--relay", "h", "stray"}, "unknown argument 'stray'");
    usage({"--relay", "h", "--json=yes"}, "unknown argument");
}

TEST_CASE("relay probe: the relay comes from the command line or WPIEMU_RELAY_PROBE_RELAY") {
    const auto env = [](const std::string& name) -> std::optional<std::string> {
        return name == wr::kProbeRelayEnvironment ? std::optional<std::string>("env.example:9000") : std::nullopt;
    };
    const auto none = [](const std::string&) -> std::optional<std::string> { return std::nullopt; };

    auto fromEnv = wr::parseProbeArgs({}, env);
    REQUIRE(fromEnv.ok());
    CHECK(fromEnv.value().options.relayHost == "env.example");
    CHECK(fromEnv.value().options.relayPort == 9000);

    auto flagWins = wr::parseProbeArgs({"--relay", "flag.example"}, env);
    REQUIRE(flagWins.ok());
    CHECK(flagWins.value().options.relayHost == "flag.example");
    CHECK(flagWins.value().options.relayPort == wr::kDefaultRelayPort);

    auto equalsForm = wr::parseProbeArgs({"--relay=a.example:1234", "--timeout-ms=2500", "--count=3", "--payload-bytes=0"}, none);
    REQUIRE(equalsForm.ok());
    CHECK(equalsForm.value().options.relayPort == 1234);
    CHECK(equalsForm.value().options.timeout == 2500ms);
    CHECK(equalsForm.value().options.count == 3);
    CHECK(equalsForm.value().options.payloadBytes == 0);

    auto v6 = wr::parseProbeArgs({"--relay", "[::1]:8000"}, none);
    REQUIRE(v6.ok());
    CHECK(v6.value().options.relayHost == "::1");
    CHECK(v6.value().options.relayPort == 8000);
    auto bareV6 = wr::parseProbeArgs({"--relay", "2001:db8::7"}, none);
    REQUIRE(bareV6.ok());
    CHECK(bareV6.value().options.relayHost == "2001:db8::7");
    CHECK(bareV6.value().options.relayPort == wr::kDefaultRelayPort);

    auto defaults = wr::parseProbeArgs({"--relay", "h"}, none);
    REQUIRE(defaults.ok());
    CHECK(defaults.value().options.mode == wr::ProbeMode::Full);
    CHECK(defaults.value().options.timeout == 10s);
    CHECK(defaults.value().options.count == 20);
    CHECK(defaults.value().options.payloadBytes == 64 * 1024);
    CHECK_FALSE(defaults.value().options.json);
    CHECK_FALSE(defaults.value().options.printInvite);
}

TEST_CASE("relay probe: help and version succeed without a relay") {
    const CliRun help = cli({"--help"});
    CHECK(help.code == 0);
    CHECK(contains(help.out, "--relay HOST[:PORT]"));
    CHECK(contains(help.out, wr::kProbeRelayEnvironment));
    for (const char* flag : {"--mode", "--timeout-ms", "--count", "--payload-bytes", "--invite", "--print-invite", "--json", "--version"})
        CHECK_MESSAGE(contains(help.out, flag), flag);
    const CliRun version = cli({"--version"});
    CHECK(version.code == 0);
    CHECK(contains(version.out, "wpiemu_relay_probe"));
    CHECK(contains(version.out, "relay protocol"));
}

TEST_CASE("relay probe: percentiles use the nearest rank") {
    CHECK(wr::percentileOfSorted({}, 0.95) == 0);
    CHECK(wr::percentileOfSorted({7}, 0.95) == 7);
    std::vector<double> twenty;
    for (int i = 1; i <= 20; ++i) twenty.push_back(i);
    CHECK(wr::percentileOfSorted(twenty, 0.95) == 19);
    CHECK(wr::percentileOfSorted(twenty, 1.0) == 20);
    CHECK(wr::percentileOfSorted(twenty, 0.0) == 1);
}

// ---- against a real relay -------------------------------------------------------------------------------------------------

TEST_CASE("relay probe: the full probe passes against the real relay and leaves a clean room behind") {
    StackRelay relay;
    REQUIRE(relay.started());
    const CliRun run = cli({"--relay", relayArg(relay.port()), "--count", "8", "--payload-bytes", "70000", "--timeout-ms", "8000"});
    CHECK_MESSAGE(run.code == 0, run.out << run.err);
    for (const char* step : {"reach", "invite", "register", "join", "handshake", "reliable-echo", "unreliable-echo", "payload", "leave"})
        CHECK_MESSAGE(contains(run.out, step), step);
    CHECK(contains(run.out, "RESULT: ok"));
    CHECK(run.err.empty());
    CHECK(run.elapsed < 8s);

    // One room, one client, and both are gone again at once: the probe's host sent CloseRoom, nothing waits for the TTL.
    REQUIRE(waitFor([&] { return relay.stats().roomsClosed == 1 && relay.stats().rooms == 0 && relay.stats().peers == 0; }));
    const wr::RelayStats stats = relay.stats();
    CHECK(stats.roomsWithoutHost == 0);
    CHECK(stats.pingsAnswered == 1);
    CHECK(stats.roomsCreated == 1);
    CHECK(stats.peersJoined == 1);
    CHECK(stats.framesDroppedTotal() == 0);
    CHECK(stats.registersRejected == 0);
    CHECK(relay.logs->contains("room_created"));
}

TEST_CASE("relay probe: the JSON report has the documented shape") {
    StackRelay relay;
    REQUIRE(relay.started());
    const CliRun run = cli({"--relay", relayArg(relay.port()), "--json", "--count", "6", "--payload-bytes", "4096"});
    REQUIRE_MESSAGE(run.code == 0, run.out << run.err);
    CHECK(run.err.empty());
    const Json report = parseJsonLine(run.out);

    CHECK(member(report, "ok").type == Json::Type::Bool);
    CHECK(member(report, "ok").boolean);
    CHECK(member(report, "mode").text == "full");
    CHECK(member(report, "relay").text == relayArg(relay.port()));
    CHECK(member(report, "resolvedAddress").text == relayArg(relay.port()));
    const Json rtt = member(report, "rttMs");
    REQUIRE(rtt.type == Json::Type::Object);
    for (const char* key : {"min", "avg", "p95", "max"}) {
        CHECK_MESSAGE(member(rtt, key).type == Json::Type::Number, key);
        CHECK_MESSAGE(member(rtt, key).number > 0, key);
    }
    CHECK(member(rtt, "min").number <= member(rtt, "avg").number);
    CHECK(member(rtt, "avg").number <= member(rtt, "max").number);
    CHECK(member(rtt, "p95").number <= member(rtt, "max").number);
    CHECK(member(report, "unreliableLossPermille").type == Json::Type::Number);
    CHECK(member(report, "unreliableLossPermille").number <= 1000);
    CHECK(member(report, "reliableBytes").number == 4096);
    CHECK(report.get("error") == nullptr);
    CHECK(report.get("invite") == nullptr);

    const Json steps = member(report, "steps");
    REQUIRE(steps.type == Json::Type::Array);
    std::vector<std::string> names;
    for (const Json& step : steps.items) {
        REQUIRE(step.type == Json::Type::Object);
        CHECK(member(step, "ok").boolean);
        CHECK(member(step, "ms").type == Json::Type::Number);
        CHECK(member(step, "ms").number >= 0);
        CHECK_FALSE(member(step, "detail").text.empty());
        names.push_back(member(step, "name").text);
    }
    CHECK(names == std::vector<std::string>{"reach", "invite", "register", "join", "handshake", "reliable-echo", "unreliable-echo", "payload", "leave"});
}

TEST_CASE("relay probe: the invite secret is never printed unless asked for, and the relay never sees it") {
    StackRelay relay;
    REQUIRE(relay.started());
    const Invite invite = newInvite();
    const std::string text = net::inviteText(invite.secret);
    std::string bare = text;
    bare.erase(std::remove(bare.begin(), bare.end(), '-'), bare.end());

    for (const bool json : {false, true}) {
        std::vector<std::string> args{"--relay", relayArg(relay.port()), "--invite", text, "--count", "2", "--payload-bytes", "100"};
        if (json) args.push_back("--json");
        const CliRun run = cli(args);
        REQUIRE_MESSAGE(run.code == 0, run.out << run.err);
        CHECK_FALSE(contains(run.out, text));
        CHECK_FALSE(contains(run.err, text));
        CHECK_FALSE(contains(run.out, bare));
        CHECK_FALSE(contains(run.out, hexOf(invite.secret.bytes())));
        CHECK(contains(run.out, invite.room));  // the derived room id may be shown
        // The probe's host sent CloseRoom, so the room is gone at once and the same invite can be used again.
        REQUIRE(waitFor([&] { return relay.stats().rooms == 0; }));
    }
    CHECK_FALSE(relay.logs->contains(text));
    CHECK_FALSE(relay.logs->contains(bare));
    CHECK_FALSE(relay.capture->anyContains(std::span<const std::byte>(reinterpret_cast<const std::byte*>(invite.secret.bytes().data()),
                                                                      invite.secret.bytes().size())));

    SUBCASE("--print-invite prints it, in text and in JSON") {
        const CliRun printed = cli({"--relay", relayArg(relay.port()), "--print-invite", "--count", "1", "--payload-bytes", "0"});
        REQUIRE_MESSAGE(printed.code == 0, printed.out << printed.err);
        const auto start = printed.out.find("\n  invite ");
        REQUIRE(start != std::string::npos);
        CHECK(wpiemu::net::parseInvite(printed.out.substr(start + 10, 23)).ok());

        const CliRun printedJson = cli({"--relay", relayArg(relay.port()), "--print-invite", "--json", "--count", "1", "--payload-bytes", "0"});
        REQUIRE(printedJson.code == 0);
        const Json report = parseJsonLine(printedJson.out);
        CHECK(wpiemu::net::parseInvite(member(report, "invite").text).ok());
    }
}

TEST_CASE("relay probe: a given invite names the room, otherwise every probe makes a fresh one") {
    StackRelay relay;
    REQUIRE(relay.started());
    const Invite invite = newInvite();
    const std::vector<std::string> quick{"--json", "--count", "1", "--payload-bytes", "0", "--relay", relayArg(relay.port())};

    std::vector<std::string> given = quick;
    given.insert(given.end(), {"--invite", net::inviteText(invite.secret)});
    const CliRun first = cli(given);
    REQUIRE_MESSAGE(first.code == 0, first.out << first.err);
    CHECK(member(parseJsonLine(first.out), "room").text == invite.room);
    CHECK(relay.stats().roomsCreated == 1);

    const CliRun second = cli(quick);
    REQUIRE_MESSAGE(second.code == 0, second.out << second.err);
    const std::string freshRoom = member(parseJsonLine(second.out), "room").text;
    CHECK(wpiemu::sdk::match::relay::isValidRoomId(freshRoom));
    CHECK(freshRoom != invite.room);
    CHECK(relay.stats().roomsCreated == 2);  // at most one room per probe
}

TEST_CASE("relay probe: reach and selfcheck create nothing") {
    StackRelay relay;
    REQUIRE(relay.started());
    for (const char* mode : {"reach", "selfcheck"}) {
        const CliRun run = cli({"--relay", relayArg(relay.port()), "--mode", mode, "--json"});
        REQUIRE_MESSAGE(run.code == 0, run.out << run.err);
        const Json report = parseJsonLine(run.out);
        CHECK(member(report, "ok").boolean);
        CHECK(member(report, "mode").text == mode);
        CHECK(member(report, "steps").items.size() == 1);
        CHECK(member(report, "reliableBytes").number == 0);
        CHECK(member(member(report, "rttMs"), "avg").number == 0);
    }
    CHECK(relay.stats().roomsCreated == 0);
    CHECK(relay.stats().peersJoined == 0);
    CHECK(relay.stats().joinsRejected == 0);  // the health-ping room is answered, not counted as a failed join
    CHECK(relay.stats().pingsAnswered == 2);
}

TEST_CASE("relay probe: the count and the payload can be switched off") {
    StackRelay relay;
    REQUIRE(relay.started());
    const CliRun run = cli({"--relay", relayArg(relay.port()), "--json", "--count", "0", "--payload-bytes", "0"});
    REQUIRE_MESSAGE(run.code == 0, run.out << run.err);
    const Json report = parseJsonLine(run.out);
    std::vector<std::string> names;
    for (const Json& step : member(report, "steps").items) names.push_back(member(step, "name").text);
    CHECK(names == std::vector<std::string>{"reach", "invite", "register", "join", "handshake", "leave"});
}

TEST_CASE("relay probe: the largest reliable payload makes the round trip") {
    StackRelay relay;
    REQUIRE(relay.started());
    const CliRun run = cli({"--relay", relayArg(relay.port()), "--json", "--count", "2", "--payload-bytes", std::to_string(wr::kProbeMaxPayloadBytes)});
    REQUIRE_MESSAGE(run.code == 0, run.out << run.err);
    CHECK(member(parseJsonLine(run.out), "reliableBytes").number == static_cast<double>(wr::kProbeMaxPayloadBytes));
    CHECK(relay.stats().framesDroppedTotal() == 0);
}

TEST_CASE("relay probe: reach never counts toward the lockout, a locked out source is reported as such") {
    StackRelay relay([](wr::RelayOptions& o) {
        o.server.exemptLoopbackFromLockout = false;
        o.server.table.lockoutMaxFailures = 2;
    });
    REQUIRE(relay.started());
    // Far more probes than the lockout allows: the ping room is not a failed join.
    for (int i = 0; i < 6; ++i) CHECK(cli({"--relay", relayArg(relay.port()), "--mode", "reach"}).code == 0);
    CHECK(relay.stats().lockouts == 0);
    CHECK(relay.stats().pingsAnswered == 6);

    // Real failed joins of this source do lock it out, and the probe says so.
    for (int i = 0; i < 2; ++i) CHECK_FALSE(joinRaw(relay, "ZZZZZZZZ").ok());
    REQUIRE(waitFor([&] { return relay.stats().lockouts == 1; }));
    const CliRun locked = cli({"--relay", relayArg(relay.port()), "--mode", "reach"});
    CHECK(locked.code == 1);
    CHECK_MESSAGE(contains(locked.out, "locked"), locked.out);
}

// ---- when the relay is not there or not a relay ------------------------------------------------------------------------------

TEST_CASE("relay probe: a closed port fails within the deadline and names UDP") {
    std::uint16_t deadPort = 0;
    {
        StackRelay gone;
        REQUIRE(gone.started());
        deadPort = gone.port();
    }
    for (const char* mode : {"reach", "full", "selfcheck"}) {
        const CliRun run = cli({"--relay", relayArg(deadPort), "--mode", mode, "--timeout-ms", "700"});
        CHECK_MESSAGE(run.code == 1, mode);
        CHECK_MESSAGE((contains(run.out, "no UDP response") || contains(run.out, "no connection")), run.out);
        if (std::string(mode) != "selfcheck") CHECK_MESSAGE(contains(run.out, "UDP may be blocked on this network"), run.out);
        CHECK_MESSAGE(contains(run.out, "RESULT: FAILED"), run.out);
        CHECK_MESSAGE(run.elapsed < 700ms + 1500ms, run.elapsed.count());
        CHECK(contains(run.err, "wpiemu_relay_probe:"));
    }
}

TEST_CASE("relay probe: a relay that never answers fails every mode in bounded time") {
    ScriptedRelay silent(ScriptedRelay::Behaviour::Silent);
    for (const char* mode : {"reach", "full", "selfcheck"}) {
        const CliRun run = cli({"--relay", relayArg(silent.port()), "--mode", mode, "--timeout-ms", "800", "--json"});
        CHECK_MESSAGE(run.code == 1, mode);
        const Json report = parseJsonLine(run.out);
        CHECK_FALSE(member(report, "ok").boolean);
        CHECK_MESSAGE((contains(member(report, "error").text, "protocol mismatch") || contains(member(report, "error").text, "no answer")),
                      member(report, "error").text);
        CHECK_MESSAGE(run.elapsed < 800ms + 1500ms, run.elapsed.count());
    }
}

TEST_CASE("relay probe: a peer that speaks ENet but not the relay protocol is a protocol mismatch") {
    struct Case {
        ScriptedRelay::Behaviour behaviour;
        const char* expected;
    };
    const std::vector<Case> cases{
        {ScriptedRelay::Behaviour::Garbage, "protocol mismatch"},
        {ScriptedRelay::Behaviour::HangUp, "closed the connection"},
        {ScriptedRelay::Behaviour::RefuseVersion, "does not speak relay protocol"},
        {ScriptedRelay::Behaviour::AcceptJoin, "accepted a join"},
        {ScriptedRelay::Behaviour::LockedOut, "locked this network address out"},
        {ScriptedRelay::Behaviour::RoomFull, "unexpected reason"},
    };
    for (const Case& c : cases) {
        ScriptedRelay relay(c.behaviour);
        const CliRun run = cli({"--relay", relayArg(relay.port()), "--mode", "reach", "--timeout-ms", "1500"});
        CHECK_MESSAGE(run.code == 1, run.out);
        CHECK_MESSAGE(contains(run.out, c.expected), c.expected << " in: " << run.out);
        CHECK(run.elapsed < 1500ms + 1500ms);
    }
}

TEST_CASE("relay probe: selfcheck mode accepts any refusal like the relay's own liveness check") {
    ScriptedRelay relay(ScriptedRelay::Behaviour::RoomFull);
    CHECK(cli({"--relay", relayArg(relay.port()), "--mode", "selfcheck", "--timeout-ms", "1500"}).code == 0);
    CHECK(cli({"--relay", relayArg(relay.port()), "--mode", "reach", "--timeout-ms", "1500"}).code == 1);
}

TEST_CASE("relay probe: a relay that answers the reach check but never registers rooms stops the full probe at the deadline") {
    ScriptedRelay relay(ScriptedRelay::Behaviour::NoSuchRoom);
    const CliRun run = cli({"--relay", relayArg(relay.port()), "--timeout-ms", "1200", "--json"});
    CHECK(run.code == 1);
    const Json report = parseJsonLine(run.out);
    CHECK_FALSE(member(report, "ok").boolean);
    const auto& steps = member(report, "steps").items;
    REQUIRE(steps.size() >= 3);
    CHECK(member(steps.front(), "name").text == "reach");
    CHECK(member(steps.front(), "ok").boolean);
    CHECK(member(steps.back(), "name").text == "register");
    CHECK_FALSE(member(steps.back(), "ok").boolean);
    CHECK(contains(member(report, "error").text, "register"));
    CHECK(run.elapsed < 1200ms + 1500ms);
}

TEST_CASE("relay probe: a plain UDP echo is not a relay") {
    UdpEcho echo;
    const CliRun run = cli({"--relay", relayArg(echo.port()), "--mode", "reach", "--timeout-ms", "800"});
    CHECK(run.code == 1);
    CHECK_MESSAGE((contains(run.out, "UDP") || contains(run.out, "protocol mismatch")), run.out);
    CHECK(run.elapsed < 800ms + 1500ms);
    CHECK(echo.datagrams() >= 1);  // the probe did reach it
}

TEST_CASE("relay probe: the JSON report of a failure carries the error and stays one line") {
    const CliRun run = cli({"--relay", "127.0.0.1:1", "--json", "--timeout-ms", "300"});
    CHECK(run.code == 1);
    CHECK(run.err.empty());
    const Json report = parseJsonLine(run.out);
    CHECK_FALSE(member(report, "ok").boolean);
    CHECK(member(report, "error").type == Json::Type::String);
    CHECK_FALSE(member(report, "error").text.empty());
    CHECK(member(report, "steps").items.size() == 1);
    CHECK_FALSE(member(member(report, "steps").items[0], "ok").boolean);
    CHECK(report.get("resolvedAddress") == nullptr);
}

TEST_CASE("relay probe: JSON strings are escaped") {
    wr::ProbeResult result;
    result.mode = wr::ProbeMode::Reach;
    result.relay = "host\"with\\odd\nchars:1";
    result.error = "line1\nline2\t\"quoted\"";
    wr::ProbeStep step;
    step.name = "reach";
    step.detail = "a\x01 control";
    result.steps.push_back(step);
    const std::string text = wr::formatProbeJson(result) + "\n";
    const Json report = parseJsonLine(text);
    CHECK(member(report, "relay").text == result.relay);
    CHECK(member(report, "error").text == result.error);
}
