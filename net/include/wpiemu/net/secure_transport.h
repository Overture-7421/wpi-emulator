#pragma once

// End-to-end encryption for one peer connection (docs: "Transport encryption").
//
// SecureTransport IS an sdk::proto::ITransport that wraps another one, so MatchHost, MatchClientSession and MessageChannel
// use it unchanged. It sits above ENet and above the relay's peer-id envelope: the relay and anyone on the path see only
// opaque bytes. Everything is keyed from the random invite secret (invite.h); there is no identity or PKI.
//
// Handshake (blocking, bounded by one deadline, run by the factories; every frame is one raw frame on the inner transport,
// all integers little endian):
//
//   1. client -> host   "WPSC" | u8 version (1) | ephemeral X25519 public key (32)                      37 bytes
//   2. host -> client   "WPSC" | u8 version (1) | ephemeral X25519 public key (32) | confirm record      77 bytes
//   3. client -> host   confirm record                                                                  40 bytes
//
// Both sides compute DH = X25519(own ephemeral secret, peer ephemeral public) and
//   keys = BLAKE2b-512(key = channelKey(secret), message = DH | ephC | ephH | roomId(secret) as 8 ASCII chars)
// whose first 32 bytes are the client->host key and the last 32 the host->client key. A confirm record is a normal record
// (below) with counter 0, kind Confirm and the 16-byte plaintext "wpiemu-confirm-h" (host) or "wpiemu-confirm-c" (client).
// A wrong secret, a bad confirmation, a wrong magic, a wrong or unknown version, an all-zero DH result, or a frame of any
// other size closes the inner transport and fails the factory BEFORE any application frame is delivered. Send application
// frames only after the factory returned: a frame that overtook the final confirmation would fail the host's handshake.
//
// Record (one per application frame; the whole record is one frame on the inner transport):
//
//   u64 counter | ciphertext (same length as the plaintext) | 16-byte Poly1305 tag        24 bytes of overhead
//
// XChaCha20-Poly1305. The 24-byte nonce is: byte 0 = direction (0 client->host, 1 host->client), bytes 1..15 zero, bytes
// 16..23 = the counter. The associated data is two bytes: the delivery class (0 reliable, 1 unreliable) and the kind
// (0 application data, 1 confirm); the receiver learns the class by trying both. Each direction has its own counter, which
// starts at 1 for application data (0 is the confirmation); the receiver keeps a 2048-entry sliding replay window, because
// ENet can reorder unreliable against reliable frames. A short frame, a bad tag, a replayed or too old counter is dropped
// and counted in stats(), never thrown and never a disconnect by itself. sendUnreliable uses the inner sendUnreliable, so
// the delivery class survives end to end. A send counter that would wrap closes the transport instead.
//
// Threading is the ITransport contract: one sender thread, one receiver thread, close() from anywhere. The session keys
// are wiped on destruction.

#include <array>
#include <atomic>
#include <bitset>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <vector>

#include "wpiemu/net/invite.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::net {

struct SecureTransportStats {
    std::uint64_t sent = 0;               // application records sent
    std::uint64_t received = 0;           // application records decrypted and delivered
    std::uint64_t droppedShort = 0;       // shorter than a record's overhead
    std::uint64_t droppedBadTag = 0;      // failed authentication (tampered, wrong key or corrupted)
    std::uint64_t droppedReplay = 0;      // counter already seen, or older than the replay window
};

class SecureTransport final : public sdk::proto::ITransport {
public:
    static constexpr std::size_t kRecordOverhead = 24;  // u64 counter + 16-byte tag
    static constexpr std::size_t kReplayWindow = 2048;
    static constexpr std::uint8_t kVersion = 1;
    static constexpr std::size_t kHelloBytes = 37;
    static constexpr std::size_t kReplyBytes = 77;
    static constexpr std::size_t kConfirmBytes = 40;

    // The initiator (a client joining): sends the first handshake frame, then verifies the host's confirmation.
    static sdk::Result<std::unique_ptr<sdk::proto::ITransport>> connectAsClient(
        std::unique_ptr<sdk::proto::ITransport> inner, const InviteSecret& secret, std::chrono::milliseconds timeout);
    // The responder (the host, one call per accepted peer): waits for the client's first frame.
    static sdk::Result<std::unique_ptr<sdk::proto::ITransport>> acceptAsHost(
        std::unique_ptr<sdk::proto::ITransport> inner, const InviteSecret& secret, std::chrono::milliseconds timeout);

    ~SecureTransport() override;

    sdk::Status send(std::span<const std::byte> frame) override;
    sdk::Status sendUnreliable(std::span<const std::byte> frame) override;
    sdk::Result<bool> receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) override;
    void close() override;
    bool connected() const override;
    std::string describe() const override;

    SecureTransportStats stats() const;

    // Test seam: the next application record will use `counter`.
    void setNextSendCounterForTest(std::uint64_t counter);

    // Internal factory seam for tests: the same handshake with the ephemeral key bytes drawn from `fill` instead of the OS.
    using RandomFill = std::function<sdk::Status(std::span<std::byte>)>;
    static sdk::Result<std::unique_ptr<sdk::proto::ITransport>> connectAsClientWith(
        std::unique_ptr<sdk::proto::ITransport> inner, const InviteSecret& secret, std::chrono::milliseconds timeout,
        const RandomFill& fill);
    static sdk::Result<std::unique_ptr<sdk::proto::ITransport>> acceptAsHostWith(
        std::unique_ptr<sdk::proto::ITransport> inner, const InviteSecret& secret, std::chrono::milliseconds timeout,
        const RandomFill& fill);

private:
    using Key = std::array<std::uint8_t, 32>;

    SecureTransport(std::unique_ptr<sdk::proto::ITransport> inner, const Key& sendKey, const Key& recvKey,
                    std::uint8_t sendDirection);

    sdk::Status sendRecord(std::span<const std::byte> frame, std::uint8_t delivery);
    bool replayed(std::uint64_t counter) const;
    void markSeen(std::uint64_t counter);

    std::unique_ptr<sdk::proto::ITransport> inner_;
    Key sendKey_{};
    Key recvKey_{};
    std::uint8_t sendDirection_ = 0;

    std::mutex sendMutex_;
    std::uint64_t nextSendCounter_ = 1;
    bool sendExhausted_ = false;
    std::vector<std::byte> sendScratch_;

    std::vector<std::byte> recvScratch_;
    std::uint64_t recvTop_ = 0;
    std::bitset<kReplayWindow> recvSeen_;  // bit i is counter recvTop_ - i

    std::atomic<bool> closed_{false};
    std::atomic<std::uint64_t> sent_{0};
    std::atomic<std::uint64_t> received_{0};
    std::atomic<std::uint64_t> droppedShort_{0};
    std::atomic<std::uint64_t> droppedBadTag_{0};
    std::atomic<std::uint64_t> droppedReplay_{0};
};

}  // namespace wpiemu::net
