#pragma once

// Opens the encrypted link to a host: the transport the endpoint names (direct ENet or the relay), then the SecureTransport
// client handshake keyed with the invite, so the match protocol only ever runs over an authenticated, encrypted channel. The
// call blocks (it dials and waits for the host's confirmation): run it off the UI thread. A wrong invite fails the handshake
// before anything is parsed; the error says which stage failed.

#include <chrono>
#include <memory>

#include "client/join_form.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::app::client {

struct LinkOptions {
    std::chrono::milliseconds connectTimeout{5000};      // reaching the host or the relay (and the room)
    std::chrono::milliseconds handshakeTimeout{3000};    // the invite handshake
};

sdk::Result<std::unique_ptr<sdk::proto::ITransport>> openSecureLink(const net::NetEndpoint& endpoint, const net::InviteSecret& secret,
                                                                    const LinkOptions& options = {});

}  // namespace wpiemu::app::client
