#include "wpiemu/net/secure_transport.h"

#include <algorithm>
#include <cstring>

#include "monocypher.h"
#include "wpiemu/net/os_random.h"

namespace wpiemu::net {

namespace {

using Key = std::array<std::uint8_t, 32>;
using sdk::ErrorCode;
using sdk::makeError;
using sdk::proto::ITransport;

constexpr std::array<std::uint8_t, 4> kMagic{'W', 'P', 'S', 'C'};
constexpr std::uint8_t kDirClientToHost = 0;
constexpr std::uint8_t kDirHostToClient = 1;
constexpr std::uint8_t kClassReliable = 0;
constexpr std::uint8_t kClassUnreliable = 1;
constexpr std::uint8_t kKindData = 0;
constexpr std::uint8_t kKindConfirm = 1;
constexpr std::size_t kCounterBytes = 8;
constexpr std::size_t kTagBytes = 16;
constexpr std::size_t kKeyBytes = 32;
constexpr std::size_t kConfirmPlainBytes = 16;
constexpr std::array<std::uint8_t, kConfirmPlainBytes> kHostConfirm{'w', 'p', 'i', 'e', 'm', 'u', '-', 'c',
                                                                      'o', 'n', 'f', 'i', 'r', 'm', '-', 'h'};
constexpr std::array<std::uint8_t, kConfirmPlainBytes> kClientConfirm{'w', 'p', 'i', 'e', 'm', 'u', '-', 'c',
                                                                        'o', 'n', 'f', 'i', 'r', 'm', '-', 'c'};

const std::uint8_t* raw(const std::byte* p) { return reinterpret_cast<const std::uint8_t*>(p); }
std::uint8_t* raw(std::byte* p) { return reinterpret_cast<std::uint8_t*>(p); }

void putCounter(std::byte* out, std::uint64_t counter) {
    for (std::size_t i = 0; i < kCounterBytes; ++i) out[i] = static_cast<std::byte>((counter >> (8 * i)) & 0xff);
}

std::uint64_t getCounter(const std::byte* in) {
    std::uint64_t counter = 0;
    for (std::size_t i = 0; i < kCounterBytes; ++i)
        counter |= static_cast<std::uint64_t>(std::to_integer<std::uint8_t>(in[i])) << (8 * i);
    return counter;
}

std::array<std::uint8_t, 24> makeNonce(std::uint8_t direction, std::uint64_t counter) {
    std::array<std::uint8_t, 24> nonce{};
    nonce[0] = direction;
    for (std::size_t i = 0; i < kCounterBytes; ++i) nonce[16 + i] = static_cast<std::uint8_t>((counter >> (8 * i)) & 0xff);
    return nonce;
}

// out = counter | ciphertext | tag
void seal(const Key& key, std::uint8_t direction, std::uint64_t counter, std::uint8_t delivery, std::uint8_t kind,
          std::span<const std::byte> plain, std::vector<std::byte>& out) {
    out.resize(kCounterBytes + plain.size() + kTagBytes);
    putCounter(out.data(), counter);
    const auto nonce = makeNonce(direction, counter);
    const std::array<std::uint8_t, 2> ad{delivery, kind};
    crypto_aead_lock(raw(out.data() + kCounterBytes), raw(out.data() + kCounterBytes + plain.size()), key.data(),
                     nonce.data(), ad.data(), ad.size(), raw(plain.data()), plain.size());
}

// `record` is at least the overhead long. The delivery class is not on the wire: try both, reliable first.
bool open(const Key& key, std::uint8_t direction, std::span<const std::byte> record, std::uint8_t kind,
          std::vector<std::byte>& plain) {
    const std::uint64_t counter = getCounter(record.data());
    const auto nonce = makeNonce(direction, counter);
    const std::size_t textSize = record.size() - kCounterBytes - kTagBytes;
    const std::byte* cipher = record.data() + kCounterBytes;
    const std::byte* tag = cipher + textSize;
    plain.resize(textSize);
    for (const std::uint8_t delivery : {kClassReliable, kClassUnreliable}) {
        const std::array<std::uint8_t, 2> ad{delivery, kind};
        if (crypto_aead_unlock(raw(plain.data()), raw(tag), key.data(), nonce.data(), ad.data(), ad.size(), raw(cipher),
                               textSize) == 0)
            return true;
    }
    crypto_wipe(plain.data(), plain.size());
    plain.clear();
    return false;
}

struct Ephemeral {
    Key secret{};
    Key pub{};
    ~Ephemeral() { crypto_wipe(secret.data(), secret.size()); }
};

sdk::Result<Ephemeral> makeEphemeral(const SecureTransport::RandomFill& fill) {
    Ephemeral e;
    if (sdk::Status s = fill(std::as_writable_bytes(std::span(e.secret))); !s) return s.error();
    crypto_x25519_public_key(e.pub.data(), e.secret.data());
    return e;
}

struct SessionKeys {
    Key clientToHost{};
    Key hostToClient{};
    ~SessionKeys() {
        crypto_wipe(clientToHost.data(), clientToHost.size());
        crypto_wipe(hostToClient.data(), hostToClient.size());
    }
};

// Fails if the peer key is a low-order point (all-zero shared secret).
sdk::Result<SessionKeys> deriveKeys(const InviteSecret& invite, const Ephemeral& own, const Key& peerPublic,
                                    const Key& clientPublic, const Key& hostPublic) {
    std::array<std::uint8_t, 32 + 32 + 32 + sdk::match::relay::kRelayRoomIdChars> message{};
    crypto_x25519(message.data(), own.secret.data(), peerPublic.data());
    std::uint8_t anyDh = 0;
    for (std::size_t i = 0; i < 32; ++i) anyDh = static_cast<std::uint8_t>(anyDh | message[i]);
    if (anyDh == 0) {
        crypto_wipe(message.data(), message.size());
        return makeError(ErrorCode::ProtocolError, "secure handshake: invalid peer key");
    }
    std::memcpy(message.data() + 32, clientPublic.data(), 32);
    std::memcpy(message.data() + 64, hostPublic.data(), 32);
    const std::string room = roomId(invite);
    std::memcpy(message.data() + 96, room.data(), room.size());

    Key channel = channelKey(invite);
    std::array<std::uint8_t, 2 * kKeyBytes> okm{};
    crypto_blake2b_keyed(okm.data(), okm.size(), channel.data(), channel.size(), message.data(), message.size());
    SessionKeys keys;
    std::memcpy(keys.clientToHost.data(), okm.data(), kKeyBytes);
    std::memcpy(keys.hostToClient.data(), okm.data() + kKeyBytes, kKeyBytes);
    crypto_wipe(okm.data(), okm.size());
    crypto_wipe(channel.data(), channel.size());
    crypto_wipe(message.data(), message.size());
    return keys;
}

sdk::Error failHandshake(ITransport& inner, ErrorCode code, std::string message) {
    inner.close();
    return makeError(code, std::move(message));
}

// One frame before `deadline`; an error (Timeout, Disconnected...) otherwise.
sdk::Status receiveBefore(ITransport& inner, std::chrono::steady_clock::time_point deadline,
                          std::vector<std::byte>& frame) {
    for (;;) {
        sdk::Result<bool> got = inner.receive(frame, sdk::proto::timeLeft(deadline));
        if (!got) return got.error();
        if (got.value()) return sdk::okStatus();
        if (std::chrono::steady_clock::now() >= deadline)
            return makeError(ErrorCode::Timeout, "secure handshake timed out");
    }
}

// Checks magic and version, then the exact size, so a peer speaking another version is told apart from garbage.
sdk::Status checkHandshakeHeader(std::span<const std::byte> frame, std::size_t expectedSize) {
    if (frame.size() < kMagic.size() + 1 || std::memcmp(frame.data(), kMagic.data(), kMagic.size()) != 0)
        return makeError(ErrorCode::ProtocolError, "secure handshake: not a secure channel frame");
    const auto version = std::to_integer<std::uint8_t>(frame[kMagic.size()]);
    if (version != SecureTransport::kVersion)
        return makeError(ErrorCode::VersionMismatch,
                         "secure handshake: unsupported version " + std::to_string(version));
    if (frame.size() != expectedSize)
        return makeError(ErrorCode::ProtocolError, "secure handshake: frame has the wrong size");
    return sdk::okStatus();
}

Key readKey(std::span<const std::byte> frame, std::size_t offset) {
    Key key{};
    std::memcpy(key.data(), frame.data() + offset, key.size());
    return key;
}

std::vector<std::byte> confirmRecord(const Key& key, std::uint8_t direction,
                                     const std::array<std::uint8_t, kConfirmPlainBytes>& plain) {
    std::vector<std::byte> record;
    seal(key, direction, 0, kClassReliable, kKindConfirm, std::as_bytes(std::span(plain)), record);
    return record;
}

bool confirmMatches(const Key& key, std::uint8_t direction, std::span<const std::byte> record,
                    const std::array<std::uint8_t, kConfirmPlainBytes>& expected) {
    if (record.size() != SecureTransport::kConfirmBytes || getCounter(record.data()) != 0) return false;
    std::vector<std::byte> plain;
    if (!open(key, direction, record, kKindConfirm, plain)) return false;
    return plain.size() == expected.size() && constantTimeEqual({raw(plain.data()), plain.size()}, expected);
}

}  // namespace

sdk::Result<std::unique_ptr<ITransport>> SecureTransport::connectAsClientWith(std::unique_ptr<ITransport> inner,
                                                                              const InviteSecret& secret,
                                                                              std::chrono::milliseconds timeout,
                                                                              const RandomFill& fill) {
    if (!inner) return makeError(ErrorCode::InvalidArgument, "secure handshake needs a transport");
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    sdk::Result<Ephemeral> eph = makeEphemeral(fill);
    if (!eph) return failHandshake(*inner, eph.error().code, eph.error().message);

    std::array<std::byte, kHelloBytes> hello{};
    std::memcpy(hello.data(), kMagic.data(), kMagic.size());
    hello[kMagic.size()] = static_cast<std::byte>(kVersion);
    std::memcpy(hello.data() + kMagic.size() + 1, eph.value().pub.data(), 32);
    if (sdk::Status s = inner->send(hello); !s) return failHandshake(*inner, s.error().code, s.error().message);

    std::vector<std::byte> reply;
    if (sdk::Status s = receiveBefore(*inner, deadline, reply); !s)
        return failHandshake(*inner, s.error().code, s.error().message);
    if (sdk::Status s = checkHandshakeHeader(reply, kReplyBytes); !s)
        return failHandshake(*inner, s.error().code, s.error().message);

    const Key hostPublic = readKey(reply, kMagic.size() + 1);
    sdk::Result<SessionKeys> keys = deriveKeys(secret, eph.value(), hostPublic, eph.value().pub, hostPublic);
    if (!keys) return failHandshake(*inner, keys.error().code, keys.error().message);

    const std::span<const std::byte> hostConfirm = std::span(reply).subspan(kHelloBytes);
    if (!confirmMatches(keys.value().hostToClient, kDirHostToClient, hostConfirm, kHostConfirm))
        return failHandshake(*inner, ErrorCode::ProtocolError,
                             "secure handshake failed: the invite does not match the host's");

    const std::vector<std::byte> mine = confirmRecord(keys.value().clientToHost, kDirClientToHost, kClientConfirm);
    if (sdk::Status s = inner->send(mine); !s) return failHandshake(*inner, s.error().code, s.error().message);

    return std::unique_ptr<ITransport>(
        new SecureTransport(std::move(inner), keys.value().clientToHost, keys.value().hostToClient, kDirClientToHost));
}

sdk::Result<std::unique_ptr<ITransport>> SecureTransport::acceptAsHostWith(std::unique_ptr<ITransport> inner,
                                                                           const InviteSecret& secret,
                                                                           std::chrono::milliseconds timeout,
                                                                           const RandomFill& fill) {
    if (!inner) return makeError(ErrorCode::InvalidArgument, "secure handshake needs a transport");
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    std::vector<std::byte> hello;
    if (sdk::Status s = receiveBefore(*inner, deadline, hello); !s)
        return failHandshake(*inner, s.error().code, s.error().message);
    if (sdk::Status s = checkHandshakeHeader(hello, kHelloBytes); !s)
        return failHandshake(*inner, s.error().code, s.error().message);
    const Key clientPublic = readKey(hello, kMagic.size() + 1);

    sdk::Result<Ephemeral> eph = makeEphemeral(fill);
    if (!eph) return failHandshake(*inner, eph.error().code, eph.error().message);
    sdk::Result<SessionKeys> keys = deriveKeys(secret, eph.value(), clientPublic, clientPublic, eph.value().pub);
    if (!keys) return failHandshake(*inner, keys.error().code, keys.error().message);

    std::vector<std::byte> reply(kHelloBytes);
    std::memcpy(reply.data(), kMagic.data(), kMagic.size());
    reply[kMagic.size()] = static_cast<std::byte>(kVersion);
    std::memcpy(reply.data() + kMagic.size() + 1, eph.value().pub.data(), 32);
    const std::vector<std::byte> mine = confirmRecord(keys.value().hostToClient, kDirHostToClient, kHostConfirm);
    reply.insert(reply.end(), mine.begin(), mine.end());
    if (sdk::Status s = inner->send(reply); !s) return failHandshake(*inner, s.error().code, s.error().message);

    std::vector<std::byte> theirs;
    if (sdk::Status s = receiveBefore(*inner, deadline, theirs); !s)
        return failHandshake(*inner, s.error().code, s.error().message);
    if (!confirmMatches(keys.value().clientToHost, kDirClientToHost, theirs, kClientConfirm))
        return failHandshake(*inner, ErrorCode::ProtocolError,
                             "secure handshake failed: the peer's invite does not match");

    return std::unique_ptr<ITransport>(
        new SecureTransport(std::move(inner), keys.value().hostToClient, keys.value().clientToHost, kDirHostToClient));
}

sdk::Result<std::unique_ptr<ITransport>> SecureTransport::connectAsClient(std::unique_ptr<ITransport> inner,
                                                                          const InviteSecret& secret,
                                                                          std::chrono::milliseconds timeout) {
    return connectAsClientWith(std::move(inner), secret, timeout, fillRandom);
}

sdk::Result<std::unique_ptr<ITransport>> SecureTransport::acceptAsHost(std::unique_ptr<ITransport> inner,
                                                                       const InviteSecret& secret,
                                                                       std::chrono::milliseconds timeout) {
    return acceptAsHostWith(std::move(inner), secret, timeout, fillRandom);
}

SecureTransport::SecureTransport(std::unique_ptr<ITransport> inner, const Key& sendKey, const Key& recvKey,
                                 std::uint8_t sendDirection)
    : inner_(std::move(inner)), sendKey_(sendKey), recvKey_(recvKey), sendDirection_(sendDirection) {
    recvSeen_.set(0);  // counter 0 was the confirmation
}

SecureTransport::~SecureTransport() {
    crypto_wipe(sendKey_.data(), sendKey_.size());
    crypto_wipe(recvKey_.data(), recvKey_.size());
}

sdk::Status SecureTransport::send(std::span<const std::byte> frame) { return sendRecord(frame, kClassReliable); }

sdk::Status SecureTransport::sendUnreliable(std::span<const std::byte> frame) {
    return sendRecord(frame, kClassUnreliable);
}

sdk::Status SecureTransport::sendRecord(std::span<const std::byte> frame, std::uint8_t delivery) {
    if (closed_.load()) return makeError(ErrorCode::Disconnected, "secure transport is closed");
    std::lock_guard lock(sendMutex_);
    if (sendExhausted_) {
        close();
        return makeError(ErrorCode::InvalidState, "secure transport send counter exhausted");
    }
    const std::uint64_t counter = nextSendCounter_++;
    if (nextSendCounter_ == 0) sendExhausted_ = true;
    seal(sendKey_, sendDirection_, counter, delivery, kKindData, frame, sendScratch_);
    const sdk::Status status = delivery == kClassUnreliable ? inner_->sendUnreliable(sendScratch_)
                                                            : inner_->send(sendScratch_);
    if (status) ++sent_;
    return status;
}

bool SecureTransport::replayed(std::uint64_t counter) const {
    if (counter > recvTop_) return false;
    const std::uint64_t age = recvTop_ - counter;
    return age >= kReplayWindow || recvSeen_.test(static_cast<std::size_t>(age));
}

void SecureTransport::markSeen(std::uint64_t counter) {
    if (counter > recvTop_) {
        const std::uint64_t shift = counter - recvTop_;
        recvSeen_ <<= static_cast<std::size_t>(std::min<std::uint64_t>(shift, kReplayWindow));
        recvSeen_.set(0);
        recvTop_ = counter;
    } else {
        recvSeen_.set(static_cast<std::size_t>(recvTop_ - counter));
    }
}

sdk::Result<bool> SecureTransport::receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (;;) {
        if (closed_.load()) return makeError(ErrorCode::Disconnected, "secure transport is closed");
        sdk::Result<bool> got = inner_->receive(recvScratch_, sdk::proto::timeLeft(deadline));
        if (!got) return got;
        if (got.value()) {
            if (recvScratch_.size() < kRecordOverhead) {
                ++droppedShort_;
            } else if (const std::uint64_t counter = getCounter(recvScratch_.data()); replayed(counter)) {
                ++droppedReplay_;
            } else if (!open(recvKey_, static_cast<std::uint8_t>(1 - sendDirection_), recvScratch_, kKindData, frame)) {
                ++droppedBadTag_;
            } else {
                markSeen(counter);
                ++received_;
                return true;
            }
        }
        if (std::chrono::steady_clock::now() >= deadline) return false;
    }
}

void SecureTransport::close() {
    closed_ = true;
    inner_->close();
}

bool SecureTransport::connected() const { return !closed_.load() && inner_->connected(); }

std::string SecureTransport::describe() const { return "secure(" + inner_->describe() + ")"; }

SecureTransportStats SecureTransport::stats() const {
    SecureTransportStats s;
    s.sent = sent_.load();
    s.received = received_.load();
    s.droppedShort = droppedShort_.load();
    s.droppedBadTag = droppedBadTag_.load();
    s.droppedReplay = droppedReplay_.load();
    return s;
}

void SecureTransport::setNextSendCounterForTest(std::uint64_t counter) {
    std::lock_guard lock(sendMutex_);
    nextSendCounter_ = counter;
    sendExhausted_ = counter == 0;
}

}  // namespace wpiemu::net
