#pragma once

#include <memory>

#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::halsim {

// The one place that names the concrete wpiemu_ipc transport and codec (composition root helper).
struct IpcBindings {
    std::unique_ptr<sdk::proto::ITransportClient> client;
    std::shared_ptr<const sdk::proto::IMessageCodec> codec;
};

IpcBindings makeIpcBindings();

}  // namespace wpiemu::halsim
