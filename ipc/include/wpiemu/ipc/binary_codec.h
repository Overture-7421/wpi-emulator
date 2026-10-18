#pragma once

// Concrete IMessageCodec: compact, little-endian, versioned binary serialisation of every
// sdk proto:: message (see ipc/README.md for the byte layout).
//
//   frame = u8 wireFormat | u16 messageType | body
//
// Evolution rules (match sdk/ipc/messages.h): fields are appended at the END of a message or
// record; a decoder defaults absent trailing fields (older peer) and ignores unread trailing bytes
// (newer peer). Nested records (DeviceDescriptor, SignalSpec, JoystickState) are length-prefixed so
// they can grow too; the vision records at the end of HelloAck / SensorFrame / ActuatorFrame are
// fixed layouts (ipc/README.md, "Vision tails"). Decoding is bounds-checked and allocation-limited:
// it never trusts a length.
//
// Decode error contract:
//   ProtocolError   malformed / truncated / out-of-range content (the stream is not trustworthy)
//   VersionMismatch the frame's wire-format byte is not ours
//   Unsupported     well-formed frame of an UNKNOWN message type: SKIP it and keep going
//                   (receiveSkippingUnknown() does exactly that)

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::ipc {

inline constexpr std::uint8_t kWireFormatVersion = 1;
inline constexpr std::size_t kDefaultMaxFrameBytes = std::size_t{8} << 20;
inline constexpr std::size_t kMaxStringBytes = std::size_t{1} << 20;

// Element caps of the vision vectors. The encoder refuses a message above them (InvalidArgument) so
// a peer is never sent something its decoder would reject as a protocol error.
inline constexpr std::size_t kMaxVisionFrames = 1024;        // SensorFrame::vision
inline constexpr std::size_t kMaxVisionObservations = 1024;  // VisionFrame::observations
inline constexpr std::size_t kMaxVisionTagIds = 1024;        // MultiTagSolution::tagIds
inline constexpr std::size_t kMaxVisionCameras = 256;        // HelloAck / ActuatorFrame cameras

class BinaryMessageCodec final : public sdk::proto::IMessageCodec {
public:
    explicit BinaryMessageCodec(std::size_t maxFrameBytes = kDefaultMaxFrameBytes) : maxFrameBytes_(maxFrameBytes) {}

    std::string_view name() const override { return "wpiemu-binary-1"; }
    sdk::Status encode(const sdk::proto::Message& message, std::vector<std::byte>& out) const override;
    sdk::Result<sdk::proto::Message> decode(std::span<const std::byte> frame) const override;

private:
    std::size_t maxFrameBytes_;
};

// True when `error` came from a well-formed frame of an unknown message type (safe to skip).
inline bool isSkippableDecodeError(const sdk::Error& error) { return error.code == sdk::ErrorCode::Unsupported; }

// MessageChannel::receive that silently skips unknown message types (protocol evolution). Returns
// true when a message arrived, false on timeout; `skipped` (optional) counts the skipped frames.
sdk::Result<bool> receiveSkippingUnknown(sdk::proto::MessageChannel& channel, sdk::proto::Message& out,
                                         std::chrono::milliseconds timeout, std::size_t* skipped = nullptr);

}  // namespace wpiemu::ipc
