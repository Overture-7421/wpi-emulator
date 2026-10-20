#pragma once

// IPC abstractions in ipc-facing form. Implementations live in ipc/ (named pipe / unix socket /
// shared memory transports, flatbuffers or binary codec); consumers (app/core, halsim_ext, tests)
// depend only on these interfaces.
//
// Layering:   Message  --IMessageCodec-->  bytes (one "frame")  --ITransport-->  peer
// Framing (length prefix, ring buffer slot...) is the TRANSPORT's job: send()/receive() always
// move exactly one codec frame.

#include <chrono>
#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::sdk::proto {

class IMessageCodec {
    WPIEMU_INTERFACE(IMessageCodec)
public:
    virtual std::string_view name() const = 0;  // "binary-v1", "flatbuffers"...
    // Appends one encoded frame to `out` (cleared by the caller if desired).
    virtual Status encode(const Message& message, std::vector<std::byte>& out) const = 0;
    // ProtocolError for malformed/unknown frames; never throws.
    virtual Result<Message> decode(std::span<const std::byte> frame) const = 0;
};

// Bidirectional, ordered, reliable, message-framed connection to ONE peer.
// Threading: one thread may call send() while another calls receive(); each direction is
// otherwise single-threaded. close() may be called from any thread and unblocks receive().
class ITransport {
    WPIEMU_INTERFACE(ITransport)
public:
    virtual Status send(std::span<const std::byte> frame) = 0;
    // Blocks up to `timeout`. Returns true and fills `frame` when a frame arrived, false on
    // timeout, Disconnected error when the peer is gone.
    virtual Result<bool> receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) = 0;
    virtual void close() = 0;
    virtual bool connected() const = 0;
    virtual std::string describe() const = 0;  // for logs: "pipe:\\.\pipe\wpiemu-1234"
};

// Emulator-side acceptor (the app listens, the extension connects; endpoint is passed to the
// robot via WPIEMU_ENDPOINT env, docs/reference/extension-ipc.md).
class ITransportServer {
    WPIEMU_INTERFACE(ITransportServer)
public:
    virtual Status listen() = 0;
    virtual std::string endpoint() const = 0;
    // Returns nullptr on timeout.
    virtual Result<std::unique_ptr<ITransport>> accept(std::chrono::milliseconds timeout) = 0;
    virtual void close() = 0;
};

// Extension-side connector.
class ITransportClient {
    WPIEMU_INTERFACE(ITransportClient)
public:
    virtual Result<std::unique_ptr<ITransport>> connect(const std::string& endpoint,
                                                        std::chrono::milliseconds timeout) = 0;
};

// Codec + transport glued into a typed message pipe (header-only convenience used by both ends).
class MessageChannel {
public:
    MessageChannel(std::unique_ptr<ITransport> transport, std::shared_ptr<const IMessageCodec> codec)
        : transport_(std::move(transport)), codec_(std::move(codec)) {}

    Status send(const Message& message) {
        scratch_.clear();
        if (Status s = codec_->encode(message, scratch_); !s) return s;
        return transport_->send(scratch_);
    }

    // Returns true and fills `out` when a message arrived, false on timeout, an error otherwise.
    Result<bool> receive(Message& out, std::chrono::milliseconds timeout) {
        Result<bool> got = transport_->receive(inbox_, timeout);
        if (!got || !got.value()) return got;
        Result<Message> decoded = codec_->decode(inbox_);
        if (!decoded) return decoded.error();
        out = std::move(decoded).value();
        return true;
    }

    bool connected() const { return transport_->connected(); }
    void close() { transport_->close(); }
    ITransport& transport() { return *transport_; }

private:
    std::unique_ptr<ITransport> transport_;
    std::shared_ptr<const IMessageCodec> codec_;
    std::vector<std::byte> scratch_;  // send side
    std::vector<std::byte> inbox_;    // receive side
};

}  // namespace wpiemu::sdk::proto
