#include "wpiemu/ipc/local_ipc.h"

#include "platform/platform_ipc.h"

namespace wpiemu::ipc {

using namespace sdk;

Result<std::unique_ptr<proto::ITransportServer>> makeLocalServer(const ConnectionEndpoint& endpoint,
                                                                 FramedTransportOptions options) {
    if (endpoint.kind() != platform::nativeKind())
        return makeError(ErrorCode::InvalidArgument,
                         "endpoint " + endpoint.address() + " is not a native local endpoint on this platform");
    return platform::makeNativeServer(endpoint.address(), options);
}

std::unique_ptr<proto::ITransportClient> makeLocalClient(FramedTransportOptions options) {
    return platform::makeNativeClient(options);
}

}  // namespace wpiemu::ipc
