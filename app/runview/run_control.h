#pragma once

// Pure presentation logic: which toolbar actions are enabled and how the connection / process state is
// described to the user. No widgets here; everything is unit-tested.

#include <QString>

#include "run_backend.h"

namespace wpiemu::app::runview {

struct RunControlState {
    bool canRestart = false;   // the only run control: reboots the robot code and the simulation
    bool simulating = false;   // lock-step simulation is live (speed control applies)
    bool paused = false;

    bool operator==(const RunControlState&) const = default;
};

RunControlState deriveControlState(const RunStatus& status, bool paused);

enum class Tone { Neutral, Info, Success, Warning, Error };

struct StatusPresentation {
    Tone tone = Tone::Neutral;
    QString chipText;   // short, for the status chip
    QString title;      // viewport overlay headline
    QString detail;     // viewport overlay body / chip tooltip
    bool showOverlay = false;   // cover the viewport (not running / failed)
    bool offerRestart = false;
    bool offerShowConsole = false;
};

// `snapshotConnected` is ISimSnapshot::robotConnected() of the newest snapshot (false when none).
StatusPresentation presentStatus(const RunStatus& status, bool snapshotConnected);

}  // namespace wpiemu::app::runview
