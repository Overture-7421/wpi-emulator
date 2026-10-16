#pragma once

// In-process byte-stream pair (a connected socket pair with no OS involved). Used by the in-process
// network and by tests; `maxReadChunk` forces partial reads so framing code is exercised.

#include <cstddef>
#include <memory>
#include <utility>

#include "wpiemu/ipc/byte_stream.h"

namespace wpiemu::ipc {

struct MemoryStreamOptions {
    std::size_t maxReadChunk = 0;  // 0 = no limit; N = each read() returns at most N bytes
};

using MemoryStreamPair = std::pair<std::unique_ptr<IByteStream>, std::unique_ptr<IByteStream>>;

MemoryStreamPair makeMemoryStreamPair(MemoryStreamOptions options = {});

}  // namespace wpiemu::ipc
