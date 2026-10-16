#include "ipc_bindings.h"

#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/ipc/local_ipc.h"

namespace wpiemu::halsim {

IpcBindings makeIpcBindings() {
    IpcBindings bindings;
    bindings.client = ipc::makeLocalClient();
    bindings.codec = std::make_shared<ipc::BinaryMessageCodec>();
    return bindings;
}

}  // namespace wpiemu::halsim
