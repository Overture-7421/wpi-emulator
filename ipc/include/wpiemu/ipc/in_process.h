#pragma once

// In-process "network": ITransportServer/ITransportClient pairs that exchange framed messages
// through memory (the full FramedTransport path, no OS objects). For tests of the app side and the
// extension side against each other, and for single-process embedding. An instance is a namespace
// of endpoints; there is no global registry.

#include <memory>
#include <string>

#include "wpiemu/ipc/framed_transport.h"
#include "wpiemu/ipc/memory_stream.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::ipc {

namespace detail {
struct InProcessState;
}

class InProcessNetwork {
public:
    static std::shared_ptr<InProcessNetwork> create(MemoryStreamOptions streamOptions = {},
                                                    FramedTransportOptions transportOptions = {});

    // `endpoint` is any "inproc:<name>" address (ConnectionEndpoint::generateInProcess()).
    std::unique_ptr<sdk::proto::ITransportServer> createServer(std::string endpoint);
    std::unique_ptr<sdk::proto::ITransportClient> createClient();

private:
    explicit InProcessNetwork(std::shared_ptr<detail::InProcessState> state) : state_(std::move(state)) {}

    std::shared_ptr<detail::InProcessState> state_;
};

}  // namespace wpiemu::ipc
