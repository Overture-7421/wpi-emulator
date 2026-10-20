#include "wpiemu/ipc/framing.h"

#include <algorithm>
#include <new>
#include <string>

#include "wpiemu/sdk/core/byte_order.h"

namespace wpiemu::ipc {

using namespace sdk;

void appendFrame(std::span<const std::byte> payload, std::vector<std::byte>& out) {
    appendLittleEndian(out, payload.size(), 4);
    out.insert(out.end(), payload.begin(), payload.end());
}

namespace {
// Header at the front of the unread bytes; false when fewer than 4 bytes are buffered.
bool peekLength(const std::vector<std::byte>& buf, std::size_t pos, std::uint32_t& length) {
    if (buf.size() - pos < kFrameHeaderBytes) return false;
    length = static_cast<std::uint32_t>(readLittleEndian(buf, pos, 4));
    return true;
}
}  // namespace

Status FrameAssembler::append(std::span<const std::byte> bytes) {
    try {
        if (readPos_ != 0 && readPos_ >= buffer_.size() / 2) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(readPos_));
            readPos_ = 0;
        }
        buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
    } catch (const std::bad_alloc&) {
        return makeError(ErrorCode::Internal, "out of memory while buffering a frame");
    }
    std::uint32_t length = 0;
    if (peekLength(buffer_, readPos_, length) && length > maxFrameBytes_)
        return makeError(ErrorCode::ProtocolError, "frame of " + std::to_string(length) + " bytes exceeds the limit");
    return okStatus();
}

Result<bool> FrameAssembler::next(std::vector<std::byte>& frame) {
    std::uint32_t length = 0;
    if (!peekLength(buffer_, readPos_, length)) return false;
    if (length > maxFrameBytes_)
        return makeError(ErrorCode::ProtocolError, "frame of " + std::to_string(length) + " bytes exceeds the limit");
    if (buffer_.size() - readPos_ - kFrameHeaderBytes < length) return false;
    const auto first = buffer_.begin() + static_cast<std::ptrdiff_t>(readPos_ + kFrameHeaderBytes);
    frame.assign(first, first + static_cast<std::ptrdiff_t>(length));
    readPos_ += kFrameHeaderBytes + length;
    if (readPos_ == buffer_.size()) {
        buffer_.clear();
        readPos_ = 0;
    }
    return true;
}

}  // namespace wpiemu::ipc
