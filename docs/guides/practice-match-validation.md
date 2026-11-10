# Practice match validation

Read this before a practice-match release, after changing the relay, `net/`, the match host or the client, or when someone
reports a connection that "works on my network". It lists what the automated end-to-end test proves, how to run it under a
bad network, and the checks **only a person can run** because they need Azure, two real computers or a restrictive network.

Background: [Match network](../reference/match-network.md) (protocol, encryption, trust model),
[Relay on Azure](relay-azure.md) (deploy, promote, probe) and the [user guide](../USER_GUIDE.md#host-a-practice-match).

## What runs without a person

### The end-to-end test

`e2e.practice_match` starts the real robot project `samples/swerve_robot/java` through the real `RobotSession` (Gradle, the
HALSim extension, the Box3D world and the 2026 plugin) with hosting in **Relay** mode against a production-configured relay on
loopback, encryption on, and scripted players built from the real `MatchClient` and the real net transports. It is opt-in like
every e2e test (it needs the WPILib 2026 JDK, the project's Gradle wrapper and the built `halsim_wpiemu`, and exits with the
ctest skip code 77 and a message when something is missing).

```powershell
cmake --build build --config Release --target wpiemu_e2e_match_tests --parallel 3
$env:WPIEMU_E2E = "1"
ctest --test-dir build -C Release -R e2e.practice_match --output-on-failure
```

One robot launch carries every phase:

| Phase | What it proves |
|---|---|
| 1 | `wpiemu_relay_probe`'s full mode passes against the relay (the same probe the deploy pipeline and you run). |
| 2 | The host registers a room; a player joins with the invite; the roster, the host's snapshot (`remoteRobots()`) and the player's view (the host's robot) show each other's robot in the same places. |
| 3 | The host's real WPILib robot drives on its own teleop code; the player drives on wheel setpoints and runs into the host's robot (it is commanded to the host's centre and stops at the chassis, because remote robots are real bodies in the host's world). |
| 4 | A player over an impaired network (see below) joins, is simulated and moves. |
| 5 | Attacks while the match runs: a wrong invite and an unknown room are refused at the handshake and counted (`handshakesFailed`); a plaintext peer never joins; a tampered record and a replayed record are dropped (a ping is answered once, not twice, and not at all when damaged) without disconnecting anyone; oversize frames are refused by the sender or the host; a flooding peer hits the relay's rate limit while the other player keeps receiving snapshots and the host's world keeps advancing. |
| 6 | A cut link makes the participant not connected but keeps its robot; **resume** returns the same participant to the same place; **kick** removes it and kills its resume token; a fresh join works. |
| 7 | Stop: the player is told `ServerClosing`, `CloseRoom` frees the room at once (not the 30 s TTL), the robot process is gone and the relay's UDP port is free. |

Environment variables (see the top of `tests/match_e2e/e2e_practice_match_test.cpp`): `WPIEMU_E2E_PROJECT`,
`WPIEMU_E2E_KEEP`, `WPIEMU_E2E_VERBOSE`, `WPIEMU_E2E_RELAY_PORT` (bind the in-process relay to a fixed port) and
`WPIEMU_E2E_RELAY` (`host:port` of a running relay, for example the deployed dev relay; the checks that read the relay's own
counters are skipped).

### The in-process impaired network

Phase 4 always runs: the player's uplink goes through `ImpairedTransport` (40 ms plus up to 20 ms jitter, 6 % loss of unreliable
frames) and its downlink is delayed 30 ms plus up to 20 ms, both around the real relay transport. The test checks that the host
simulates and moves the robot, that the client's measured round trip really includes the delay, and that the client stays Live.

### Impaired real network (partly manual)

`tools/e2e_netem.ps1` runs the same test while **clumsy** (WinDivert) drops, delays, reorders and duplicates the relay's UDP
packets on loopback. What is manual:

- Install clumsy yourself (https://jagt.github.io/clumsy/) and run PowerShell **elevated**; the script cannot do either.
- Decide whether a failure is a defect: the position checks assume a LAN. `-Profile wan` (40 ms, 2 % loss) and `lossy` should pass;
  `bad` (80 ms, 10 % loss, reordering, duplicates) is for looking at how the client copes, so read its output instead of trusting
  a pass or fail.
- Linux `tc netem` shapes Linux interfaces only; use it by running the relay under WSL2 or a container and setting
  `WPIEMU_E2E_RELAY`. Windows `New-NetQosPolicy` only throttles bandwidth and ignores loopback; the script's help shows how to use
  it against a deployed relay.

```powershell
pwsh tools/e2e_netem.ps1 -Profile wan -DryRun   # prints the filter and the commands only
pwsh tools/e2e_netem.ps1 -Profile wan
```

## What only a person can run

Tick each item and keep the output. A run counts only when **all** pass criteria hold.

### 1. Deployed dev relay probe from a normal network

After the first dev deployment (and after every relay change), from a machine on ordinary home or office Wi-Fi:

```powershell
wpiemu_relay_probe --relay wpiemu-relay-dev<suffix>.southcentralus.azurecontainer.io:7777 --mode reach
wpiemu_relay_probe --relay wpiemu-relay-dev<suffix>.southcentralus.azurecontainer.io:7777 --mode full --json
```

`<suffix>` is the DNS suffix bootstrap chose. Pass criteria: both exit 0; `full` reports every step ok, a round trip time that is
plausible for the distance (tens of milliseconds within the region), `unreliableLossPermille` of 0 or close to it, and 65536
reliable bytes round-tripped; the relay's logs (Log Analytics) show the probe's room created and closed, and no room remains
afterwards.

### 2. Dev relay probe from a UDP-restricted network

From a network that blocks outbound UDP (a school or corporate network, or a phone hotspot that filters it). If none is at hand,
simulate one on the probing machine with an **elevated** PowerShell and remove the rule afterwards:

```powershell
New-NetFirewallRule -DisplayName wpiemu-udp-block -Direction Outbound -Protocol UDP -RemotePort 7777 -Action Block
wpiemu_relay_probe --relay wpiemu-relay-dev<suffix>.southcentralus.azurecontainer.io:7777 --mode reach
Remove-NetFirewallRule -DisplayName wpiemu-udp-block
```

Pass criteria: the probe fails with exit code 1 within its timeout (10 s by default), prints `no UDP response from ...` and advises
that UDP may be blocked; it does not hang or crash. Then confirm the app's behavior on the same network: hosting in Relay mode
shows an error status with a reason (never a silent *Starting*), and the client's **Join match** shows a connection error and can
be retried. There is deliberately no TCP fallback.

### 3. Production promotion dry run

Promotion needs the digest that passed verification in dev. Run `relay-deploy` with `workflow_dispatch` on `main` for `prod`
with that digest, and stop at the approval to review it; then reject the pending `relay-prod` approval, or approve it when you
want the promotion for real. Check before approving: the `plan` job accepted the ref and found the dev verification record
(a digest without one must be refused), the digest in the summary is the one you intended, and the prod job will import (not
rebuild) it. After a real promotion, repeat the probe against the prod FQDN. Pass criteria: the run reaches the approval gate
without errors, the refusal cases (a branch other than `main`, an unverified digest) are refused, and after a real promotion the
probe passes against prod.

### 4. Two real machines: remote robots in Quick 3D

Use two computers on different networks if possible (one behind a phone hotspot is a good second network).

1. Machine A: start the app with a robot project, open **View > Practice match**, choose **Relay**, restart the robot program,
   copy the invite when the panel shows **Lobby**.
2. Machine B: start `wpiemu_client`, paste the invite, choose a `robot.json`, **Join match**.
3. On A, enable the Driver Station in teleop and drive. On B drive with a gamepad or keyboard. Drive the two robots into each other.
4. Look at the Quick 3D views on both machines. Offscreen tests do not render Quick 3D, so this is the only rendering check.

Pass criteria: the remote robot's chassis, bumpers and wheels appear on A and the host's robot appears on B, with wheels spinning and
steering; the robots collide and push each other instead of passing through; B's own robot responds to input at once; other
robots look smooth (no stutter at 60 Hz); the **Connection stats** overlay (`F3`) shows a ping close to the real round trip and
a small prediction error; pausing on A shows *Paused* on B; kicking B shows the removal reason on B and removes the robot from A's
view; quitting A shows *host closed the match* on B; B's **Reconnect** after unplugging the network for a few seconds returns to the
same robot within 30 s.

### 5. Firewall prompts

On a clean Windows account (or after removing the app's firewall rules), the first launch of each mode:

- **Relay:** neither the app nor `wpiemu_client` should raise a Windows Defender Firewall prompt (outbound UDP only).
- **Direct:** the app raises one prompt when it starts listening on UDP 47300. Allowing it lets a LAN client join with the host
  address and invite; denying it makes the client's join fail within its timeout with a reason, not hang.

Pass criteria: exactly the prompts above and no others, and a denied prompt leaves the app usable (hosting shows an error or the
client reports a failed join).

### 6. Reporting

Record for each item: date, machine and network, the relay build digest, the command and its output (the probe's JSON line is
enough for 1 and 2), and a screenshot for 4. File a defect for any criterion that fails, with the e2e output of the same build when
the problem reproduces on loopback.
