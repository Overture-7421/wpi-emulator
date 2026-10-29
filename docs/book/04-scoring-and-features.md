# Weeks 4–5: the referee and the extension roadmap

**In 30 seconds:** first finish a small, observable referee. Keep unimplemented rules visibly
separate from that working result; a plausible paragraph is not a feature.

## 1. What we implement now

The next workshops implement continuous switch/scale ownership scoring, a simplified once-only
auto-line bonus, disabled/expired-time guards, inventory and release counters, FMS data and HUD.
The robot physically releases a cube; the plate must detect a settled cube before any ownership
points accrue. The robot's roller can request release, but cannot request points.

For our two balances at the switches, only the home alliance can score that switch. The scale
can score for either alliance. Equal cube counts mean no ownership. Keep that distinction explicit;
awarding points for owning the opponent's switch is an easy mistake in a generic pair loop.

Snapshots copy scores and entity positions for the UI. No Qt code belongs in the rules. Declare
zone kinds, config properties, game values and HUD widgets through the SDK; the app already knows
how to edit and display them. QML owns the NWU-to-scene conversion.

## 2. Let a generic motor activate an interaction

The example's `roller` is a generic rotary mechanism mapped to PWM 2. Its measured simulated
velocity activates a chassis-mounted release zone. That zone resolves its attachment, evaluates
the SDK conditions, and lets the held cube re-enter physics at the muzzle pose.

Pickup uses a volume instead: it transforms active cube centers into the zone frame and takes one.
Held inventory is published as both HUD and a numeric game value, so a later generic BeamBreak
could consume it without a custom HAL device. The provided match fixture has only a release zone;
focused physics tests exercise pickup separately.

**Student asks:** “Could I just call release from robot code?” Not without inventing a non-WPILib
side channel. A normal motor plus an activation condition tests the same boundary a real intake uses.

## 3. Extension work: vaults, power-ups and official scorekeeping

**Not implemented in the listings.** A complete rules implementation needs a score ledger that
separates autonomous awards, teleop awards, vault cubes, parking, climbing and penalties.
Replace continuous integration with the manual's actual award timing before claiming official
scores. Test acquisition, loss, re-acquisition and phase boundaries, not just a steady scoring rate.

Then model the exchange-to-vault transfer, capacity, allocation and once-per-match activation rules.
Force and Boost need explicit target and simulation-time expiry; Levitate needs endgame accounting.
Read the [manual linked in the rule card](appendix-power-up.md) for precise eligibility and cube
handling. This guide does not provide working code or tests for these rules yet.

Ranking points also require qualification context and alliance-level conditions that a single
robot does not establish. Do not display a practice ranking point merely because you can add a
HUD key. Human feeding and opponent actions need their own declared scenario inputs and tests.

## 4. Extension work: a climber and richer presentation

**Not implemented in the listings.** The SDK has no general joint solver. A credible hanging
robot needs a hook/contact or explicitly simplified rule trigger, load/support handling, and
reset behavior. Keep the physical support condition separate from the score award so the physical
model can improve without rewriting the ledger. Touching a tower is not automatically a climb.

Custom editor panels and sound cues are also optional later additions. Prefer the generic property
inspector first; declare sounds through the SDK and request them once per transition. The app owns
playback, translation and Qt lifecycle. The [game-plugin guide](../guides/game-plugins.md) documents
these seams; this prototype does not implement a sound asset or custom Qt panel.

**You now have:** a clear boundary between the runnable lesson and the unfinished full-game work.
Continue with the actual referee source, then interactions and registration.

Next: [Workshop—write the tick, scoreboard and immutable snapshots](04a-match-and-snapshots.md).
