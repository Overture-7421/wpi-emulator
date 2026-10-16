#pragma once

// Pure presentation logic: which toolbar actions are enabled and how the connection / process state is
// described to the user. No widgets here; everything is unit-tested.

#include <QString>

#include "run_backend.h"

namespace wpiemu::app::runview {

struct RunControlState {
    bool canStart = false;
    bool canStop = false;
    bool canRestart = false;
    bool canReset = false;
    bool canPause = false;   // pause or resume toggle
    bool canStep = false;
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
    bool offerStart = false;
    bool offerRestart = false;
    bool offerShowConsole = false;
};

// `snapshotConnected` is ISimSnapshot::robotConnected() of the newest snapshot (false when none).
StatusPresentation presentStatus(const RunStatus& status, bool snapshotConnected);

QString realDsText(RealDsState state);
Tone realDsTone(RealDsState state);

}  // namespace wpiemu::app::runview
