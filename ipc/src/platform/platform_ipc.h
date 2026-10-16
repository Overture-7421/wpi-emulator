#pragma once

// Internal (not installed): the per-OS pieces behind local_ipc.h. Implemented by exactly one of
// platform/named_pipe_windows.cpp or platform/unix_socket_posix.cpp.

#include <memory>
#include <string>

#include "wpiemu/ipc/endpoint.h"
#include "wpiemu/ipc/framed_transport.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::ipc::platform {

EndpointKind nativeKind();
// Address for a freshly generated session token (hex string).
std::string nativeAddressFor(const std::string& token);
std::unique_ptr<sdk::proto::ITransportServer> makeNativeServer(std::string address, FramedTransportOptions options);
std::unique_ptr<sdk::proto::ITransportClient> makeNativeClient(FramedTransportOptions options);

}  // namespace wpiemu::ipc::platform
