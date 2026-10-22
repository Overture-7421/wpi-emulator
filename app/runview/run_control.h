#pragma once

// Pure presentation logic: which toolbar actions are enabled and how the connection / process state is
// described to the user. No widgets here; everything is unit-tested.

#include <QString>

#include "contract/runtime_host.h"

namespace wpiemu::app::runview {

struct RunControlState {
    bool canRestart = false;   // the only run control: reboots the robot code and the simulation
    bool simulating = false;   // lock-step simulation is live (speed control applies)
    bool paused = false;

    bool operator==(const RunControlState&) const = default;
};

RunControlState deriveControlState(const HostStatus& status, bool paused);

enum class Tone { Neutral, Info, Success, Warning, Error };

// What the overlay's extra button asks the shell to do about a failure the user can fix.
enum class ProblemAction { None = 0, FixSetup = 1, OpenSettings = 2 };

struct StatusPresentation {
    Tone tone = Tone::Neutral;
    QString chipText;   // short, for the status chip
    QString title;      // viewport overlay headline
    QString detail;     // viewport overlay body / chip tooltip
    bool showOverlay = false;   // cover the viewport (not running / failed)
    bool offerRestart = false;
    bool offerShowConsole = false;
    ProblemAction action = ProblemAction::None;
    QString actionText;   // the button's label when action != None
};

// `snapshotConnected` is ISimSnapshot::robotConnected() of the newest snapshot (false when none).
StatusPresentation presentStatus(const HostStatus& status, bool snapshotConnected);

}  // namespace wpiemu::app::runview
