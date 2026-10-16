#pragma once

// Composition-root factories for the local transports (named pipe on Windows, unix domain socket
// elsewhere). Consumers hold the sdk interfaces only (ITransportServer / ITransportClient).

#include <memory>

#include "wpiemu/ipc/endpoint.h"
#include "wpiemu/ipc/framed_transport.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::ipc {

// Emulator side. Fails with InvalidArgument when `endpoint` is not of this platform's native kind.
// Call listen() on the result before launching the robot, then accept() (again after a disconnect).
sdk::Result<std::unique_ptr<sdk::proto::ITransportServer>> makeLocalServer(const ConnectionEndpoint& endpoint,
                                                                           FramedTransportOptions options = {});

// Extension side. connect() retries until the server appears or the timeout elapses.
std::unique_ptr<sdk::proto::ITransportClient> makeLocalClient(FramedTransportOptions options = {});

}  // namespace wpiemu::ipc
