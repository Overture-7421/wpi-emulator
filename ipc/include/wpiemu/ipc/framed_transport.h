#pragma once

// ITransport over any IByteStream: length-prefixed frames, partial read/write handling, send
// timeout, receive-with-timeout, close() from any thread unblocks receive().

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <vector>

#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/ipc/byte_stream.h"
#include "wpiemu/ipc/framing.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::ipc {

struct FramedTransportOptions {
    std::size_t maxFrameBytes = kDefaultMaxFrameBytes;
    // A peer that stops reading for this long is treated as gone (the stream is closed).
    std::chrono::milliseconds sendTimeout{5000};
    std::size_t readChunkBytes = 64 * 1024;
};

class FramedTransport final : public sdk::proto::ITransport {
public:
    explicit FramedTransport(std::unique_ptr<IByteStream> stream, FramedTransportOptions options = {});
    ~FramedTransport() override;

    sdk::Status send(std::span<const std::byte> frame) override;
    sdk::Result<bool> receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) override;
    void close() override;
    bool connected() const override;
    std::string describe() const override;

private:
    std::unique_ptr<IByteStream> stream_;
    FramedTransportOptions options_;
    std::mutex sendMutex_;
    std::vector<std::byte> txBuffer_;
    FrameAssembler assembler_;
    std::vector<std::byte> rxChunk_;
    std::atomic<bool> broken_{false};
    sdk::Error lastError_;  // receive-thread only: sticky error to report once buffered frames are drained
    bool hasError_ = false;
};

}  // namespace wpiemu::ipc
