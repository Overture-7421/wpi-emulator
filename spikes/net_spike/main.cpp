#include <enet6/enet.h>
#include <monocypher.h>

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void check(bool ok, const char* what)
{
    std::printf("  [%s] %s\n", ok ? "ok" : "FAIL", what);
    if (!ok)
        ++g_failures;
}

struct Received
{
    std::vector<std::string> byChannel[2];
    bool connected = false;
    bool disconnected = false;
};

void pump(ENetHost* host, Received& r)
{
    ENetEvent ev;
    while (enet_host_service(host, &ev, 0) > 0)
    {
        switch (ev.type)
        {
        case ENET_EVENT_TYPE_CONNECT:
            r.connected = true;
            break;
        case ENET_EVENT_TYPE_RECEIVE:
            r.byChannel[ev.channelID].emplace_back(reinterpret_cast<const char*>(ev.packet->data),
                                                   ev.packet->dataLength);
            enet_packet_destroy(ev.packet);
            break;
        case ENET_EVENT_TYPE_DISCONNECT:
        case ENET_EVENT_TYPE_DISCONNECT_TIMEOUT:
            r.disconnected = true;
            break;
        default:
            break;
        }
    }
}

void pumpUntil(ENetHost* a, Received& ra, ENetHost* b, Received& rb, bool (*done)(const Received&, const Received&))
{
    for (int i = 0; i < 400 && !done(ra, rb); ++i)
    {
        pump(a, ra);
        pump(b, rb);
        enet_host_service(a, nullptr, 1);
    }
}

void send(ENetPeer* peer, std::uint8_t channel, const char* text, enet_uint32 flags)
{
    ENetPacket* p = enet_packet_create(text, std::strlen(text), flags);
    enet_peer_send(peer, channel, p);
}

void enetLoopback(ENetAddressType type, const char* ip, const char* label)
{
    std::printf("ENet loopback %s (%s)\n", label, ip);
    ENetAddress addr{};
    if (enet_address_set_host_ip(&addr, ip) != 0)
    {
        check(false, "address parse");
        return;
    }
    addr.port = 0;
    ENetHost* server = enet_host_create(type, &addr, 2, 2, 0, 0);
    ENetHost* client = enet_host_create(type, nullptr, 1, 2, 0, 0);
    check(server != nullptr && client != nullptr, "hosts created");
    if (!server || !client)
    {
        if (server)
            enet_host_destroy(server);
        if (client)
            enet_host_destroy(client);
        return;
    }

    ENetAddress bound{};
    enet_socket_get_address(server->socket, &bound);
    check(bound.port != 0, "server bound to an ephemeral port");
    ENetAddress target{};
    enet_address_set_host_ip(&target, ip);
    target.port = bound.port;

    ENetPeer* toServer = enet_host_connect(client, &target, 2, 0);
    check(toServer != nullptr, "connect started");
    Received srv, cli;
    pumpUntil(server, srv, client, cli, [](const Received& s, const Received& c) { return s.connected && c.connected; });
    check(srv.connected && cli.connected, "connected on both sides");

    ENetPeer* toClient = nullptr;
    for (std::size_t i = 0; i < server->peerCount; ++i)
        if (server->peers[i].state == ENET_PEER_STATE_CONNECTED)
            toClient = &server->peers[i];
    check(toClient != nullptr, "server sees client peer");

    if (toClient && toServer)
    {
        send(toServer, 0, "c2s-reliable", ENET_PACKET_FLAG_RELIABLE);
        send(toServer, 1, "c2s-unreliable", ENET_PACKET_FLAG_UNSEQUENCED);
        send(toClient, 0, "s2c-reliable", ENET_PACKET_FLAG_RELIABLE);
        send(toClient, 1, "s2c-unreliable", ENET_PACKET_FLAG_UNSEQUENCED);
        pumpUntil(server, srv, client, cli, [](const Received& s, const Received& c) {
            return !s.byChannel[0].empty() && !s.byChannel[1].empty() && !c.byChannel[0].empty() &&
                   !c.byChannel[1].empty();
        });
        check(srv.byChannel[0].size() == 1 && srv.byChannel[0][0] == "c2s-reliable", "server got reliable");
        check(srv.byChannel[1].size() == 1 && srv.byChannel[1][0] == "c2s-unreliable", "server got unreliable");
        check(cli.byChannel[0].size() == 1 && cli.byChannel[0][0] == "s2c-reliable", "client got reliable");
        check(cli.byChannel[1].size() == 1 && cli.byChannel[1][0] == "s2c-unreliable", "client got unreliable");

        enet_peer_disconnect(toServer, 0);
        pumpUntil(server, srv, client, cli, [](const Received& s, const Received& c) {
            return s.disconnected && c.disconnected;
        });
        check(srv.disconnected && cli.disconnected, "disconnected on both sides");
    }
    enet_host_destroy(client);
    enet_host_destroy(server);
}

using Bytes32 = std::array<std::uint8_t, 32>;

void randomBytes(std::uint8_t* out, std::size_t n)
{
    std::random_device rd;
    for (std::size_t i = 0; i < n; ++i)
        out[i] = static_cast<std::uint8_t>(rd());
}

const std::uint8_t* bytes(const std::string& s)
{
    return reinterpret_cast<const std::uint8_t*>(s.data());
}

void monocypherRoundTrip()
{
    std::printf("Monocypher\n");
    Bytes32 skA{}, skB{}, pkA{}, pkB{}, sharedA{}, sharedB{};
    randomBytes(skA.data(), skA.size());
    randomBytes(skB.data(), skB.size());
    crypto_x25519_public_key(pkA.data(), skA.data());
    crypto_x25519_public_key(pkB.data(), skB.data());
    crypto_x25519(sharedA.data(), skA.data(), pkB.data());
    crypto_x25519(sharedB.data(), skB.data(), pkA.data());
    check(sharedA == sharedB, "X25519 shared secrets match");

    // Key derivation: BLAKE2b over (shared || pkA || pkB) giving a 32-byte AEAD key.
    auto derive = [](const Bytes32& shared, const Bytes32& a, const Bytes32& b) {
        std::array<std::uint8_t, 96> in{};
        std::memcpy(in.data(), shared.data(), 32);
        std::memcpy(in.data() + 32, a.data(), 32);
        std::memcpy(in.data() + 64, b.data(), 32);
        Bytes32 key{};
        crypto_blake2b(key.data(), key.size(), in.data(), in.size());
        return key;
    };
    const Bytes32 keyA = derive(sharedA, pkA, pkB);
    const Bytes32 keyB = derive(sharedB, pkA, pkB);
    check(keyA == keyB, "BLAKE2b derived keys match");
    check(keyA != sharedA, "derived key differs from raw secret");

    std::array<std::uint8_t, 24> nonce{};
    randomBytes(nonce.data(), nonce.size());
    const std::string plain = "practice match state";
    const std::string ad = "header";
    std::vector<std::uint8_t> cipher(plain.size());
    std::array<std::uint8_t, 16> mac{};
    crypto_aead_lock(cipher.data(), mac.data(), keyA.data(), nonce.data(), bytes(ad), ad.size(), bytes(plain),
                     plain.size());
    check(std::memcmp(cipher.data(), plain.data(), plain.size()) != 0, "ciphertext differs from plaintext");

    std::vector<std::uint8_t> out(plain.size());
    const int ok = crypto_aead_unlock(out.data(), mac.data(), keyB.data(), nonce.data(), bytes(ad), ad.size(),
                                      cipher.data(), cipher.size());
    check(ok == 0 && std::memcmp(out.data(), plain.data(), plain.size()) == 0, "XChaCha20-Poly1305 round trip");

    auto tampered = cipher;
    tampered[0] ^= 1U;
    check(crypto_aead_unlock(out.data(), mac.data(), keyB.data(), nonce.data(), bytes(ad), ad.size(),
                             tampered.data(), tampered.size()) != 0,
          "tampered ciphertext rejected");
    auto badMac = mac;
    badMac[0] ^= 1U;
    check(crypto_aead_unlock(out.data(), badMac.data(), keyB.data(), nonce.data(), bytes(ad), ad.size(),
                             cipher.data(), cipher.size()) != 0,
          "tampered tag rejected");
    const std::string otherAd = "other";
    check(crypto_aead_unlock(out.data(), mac.data(), keyB.data(), nonce.data(), bytes(otherAd), otherAd.size(),
                             cipher.data(), cipher.size()) != 0,
          "tampered associated data rejected");

    crypto_wipe(skA.data(), skA.size());
    crypto_wipe(skB.data(), skB.size());
}

} // namespace

int main()
{
    if (enet_initialize() != 0)
    {
        std::printf("enet_initialize failed\n");
        return 1;
    }
    enetLoopback(ENET_ADDRESS_TYPE_IPV4, "127.0.0.1", "IPv4");
    enetLoopback(ENET_ADDRESS_TYPE_IPV6, "::1", "IPv6");
    enet_deinitialize();

    monocypherRoundTrip();

    if (g_failures == 0)
        std::printf("net spike: PASS\n");
    else
        std::printf("net spike: %d FAILURE(S)\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
