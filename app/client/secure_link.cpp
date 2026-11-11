#include "client/secure_link.h"

#include "wpiemu/net/enet_transport.h"
#include "wpiemu/net/relay_transport.h"
#include "wpiemu/net/secure_transport.h"

namespace wpiemu::app::client {

sdk::Result<std::unique_ptr<sdk::proto::ITransport>> openSecureLink(const net::NetEndpoint& endpoint, const net::InviteSecret& secret,
                                                                    const LinkOptions& options) {
    const std::string text = endpoint.format();
    sdk::Result<std::unique_ptr<sdk::proto::ITransport>> raw = sdk::makeError(sdk::ErrorCode::Internal, "no transport");
    if (endpoint.scheme == net::EndpointScheme::Relay) {
        net::RelayClientTransport relay;
        raw = relay.connect(text, options.connectTimeout);
    } else {
        net::EnetClientTransport enet;
        raw = enet.connect(text, options.connectTimeout);
    }
    if (!raw) return raw.error();
    return net::SecureTransport::connectAsClient(std::move(raw).value(), secret, options.handshakeTimeout);
}

}  // namespace wpiemu::app::client
