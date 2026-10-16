#pragma once

// Platform-neutral byte stream under FramedTransport. Implemented by the named-pipe / unix-socket
// streams and by MemoryByteStream (tests, in-process). Contract:
//   * one thread may read() while another write()s; close() may be called from ANY thread, is
//     idempotent, and unblocks both (they then fail with Disconnected);
//   * read() returns the number of bytes read (> 0), 0 on timeout, Disconnected on EOF/closed;
//   * write() writes ALL bytes or fails: Timeout means the stream state is undefined (close it).

#include <chrono>
#include <cstddef>
#include <span>
#include <string>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/result.h"

namespace wpiemu::ipc {

class IByteStream {
    WPIEMU_INTERFACE(IByteStream)
public:
    virtual sdk::Result<std::size_t> read(std::span<std::byte> buffer, std::chrono::milliseconds timeout) = 0;
    virtual sdk::Status write(std::span<const std::byte> data, std::chrono::milliseconds timeout) = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
    virtual std::string describe() const = 0;
};

}  // namespace wpiemu::ipc
