# 0019: Practice matches use a host-authoritative world

**Status:** Accepted

## Context

A driver practicing alone faces an empty field. Other people should be able to drive their own robots
on the same field without installing WPILib, and without a LAN, VPN or port forwarding. The simulation
is deterministic, lock-step with one WPILib robot, and runs a 200 Hz physics world.

## Decision

- **The host app owns the one world.** Remote robots are simulated on the host with the same chassis,
  tire and drivetrain code as the local robot, driven by the client's commanded wheel setpoints.
  Clients keep a disposable prediction world and interpolate everything else. Clients trust the host.
- **Networking is not written in-house.** ENet (the ENet6 fork, for IPv6) carries packets, private to
  `net/` and `relay/`. We add messages, thin transport adapters and game logic.
- **Encryption is end to end, host to client.** A decorator over `ITransport` uses Monocypher (X25519,
  BLAKE2b, XChaCha20-Poly1305) keyed from a random per-session invite secret. The relay sees only a
  room id, a host authentication value and opaque bytes, so it can be public and untrusted.
- **A relay on Azure Container Instances.** Everyone connects outbound to an ENet-speaking relay with
  rooms. Container Apps cannot host it: it has only HTTP and TCP ingress. ACI supports UDP.
- **Dev and prod are separate.** Each has its own resource group, registry, identity, logs and relay.
  Prod is promoted by image digest from dev behind an approval, and bootstrap creates one OIDC app per
  environment, each limited to its own resource group, so an unapproved job cannot deploy to prod.

## Rejected alternatives

- A VPN or NAT traversal: more setup for users; an ENet-speaking relay is simpler to operate.
- A transparent UDP forwarder: needs a public port per peer, and ENet cannot sit behind one.
- An in-house reliable-UDP transport: more code to test and secure than ENet.
- ENet6 encryption callbacks, DTLS or WireGuard, libsodium: fork-specific and not end to end through
  the relay, heavy or a VPN again, and a heavier build than Monocypher.
- A force-driven box for remote robots: tire slip, load transfer, motor limits and steering lag would
  differ from the local robot.

## Consequences

The host sees every client's robot description and inputs and can cheat, and the invite is the only
credential. Outbound UDP must be allowed and one relay instance has no redundancy. The match
protocol is a new SDK area with its own limits and tests. Current rules are in
[Match network](../reference/match-network.md) and the [relay guide](../guides/relay-azure.md).
