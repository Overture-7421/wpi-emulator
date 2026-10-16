#pragma once

// Length-prefixed framing shared by every byte-stream transport: u32 little-endian payload length,
// then the payload. FrameAssembler turns an arbitrary sequence of read chunks (partial reads, several
// frames in one read, a header split across reads) back into whole frames.

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "wpiemu/sdk/core/result.h"

namespace wpiemu::ipc {

inline constexpr std::size_t kFrameHeaderBytes = 4;

// Appends header + payload to `out`.
void appendFrame(std::span<const std::byte> payload, std::vector<std::byte>& out);

class FrameAssembler {
public:
    explicit FrameAssembler(std::size_t maxFrameBytes) : maxFrameBytes_(maxFrameBytes) {}

    // Buffers `bytes`. ProtocolError as soon as a header announces a frame larger than the limit:
    // the stream cannot be resynchronised and must be closed.
    sdk::Status append(std::span<const std::byte> bytes);

    // Pops the next complete frame into `frame` (replacing its content). False when none is complete yet.
    sdk::Result<bool> next(std::vector<std::byte>& frame);

    std::size_t buffered() const { return buffer_.size() - readPos_; }
    bool hasPartialFrame() const { return buffered() != 0; }

private:
    std::size_t maxFrameBytes_;
    std::vector<std::byte> buffer_;
    std::size_t readPos_ = 0;
};

}  // namespace wpiemu::ipc
