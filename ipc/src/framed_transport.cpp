#include "wpiemu/ipc/framed_transport.h"

namespace wpiemu::ipc {

using namespace sdk;

FramedTransport::FramedTransport(std::unique_ptr<IByteStream> stream, FramedTransportOptions options)
    : stream_(std::move(stream)),
      options_(options),
      assembler_(options.maxFrameBytes),
      rxChunk_(options.readChunkBytes < 16 ? 16 : options.readChunkBytes) {}

FramedTransport::~FramedTransport() { close(); }

Status FramedTransport::send(std::span<const std::byte> frame) {
    std::lock_guard lock(sendMutex_);
    if (broken_) return makeError(ErrorCode::Disconnected, "transport closed: " + stream_->describe());
    if (frame.size() > options_.maxFrameBytes)
        return makeError(ErrorCode::InvalidArgument, "frame exceeds the maximum size");
    txBuffer_.clear();
    appendFrame(frame, txBuffer_);
    Status written = stream_->write(txBuffer_, options_.sendTimeout);
    if (!written) {
        // A partially written frame would corrupt the stream: the connection is unusable.
        broken_ = true;
        stream_->close();
    }
    return written;
}

Result<bool> FramedTransport::receive(std::vector<std::byte>& frame, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (;;) {
        Result<bool> complete = assembler_.next(frame);
        if (!complete) {
            broken_ = true;
            stream_->close();
            return complete.error();
        }
        if (complete.value()) return true;
        if (hasError_) return lastError_;

        auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
        if (left.count() < 0) left = std::chrono::milliseconds{0};
        Result<std::size_t> got = stream_->read(rxChunk_, left);
        if (!got) {
            broken_ = true;
            lastError_ = got.error();
            if (assembler_.hasPartialFrame() && lastError_.code == ErrorCode::Disconnected)
                lastError_.message += " (connection closed in the middle of a frame)";
            hasError_ = true;
            return lastError_;
        }
        if (got.value() == 0) return false;  // timed out
        if (Status s = assembler_.append(std::span<const std::byte>(rxChunk_.data(), got.value())); !s) {
            broken_ = true;
            stream_->close();
            return s.error();
        }
    }
}

void FramedTransport::close() {
    broken_ = true;
    stream_->close();
}

bool FramedTransport::connected() const { return !broken_ && stream_->isOpen(); }

std::string FramedTransport::describe() const { return stream_->describe(); }

}  // namespace wpiemu::ipc
