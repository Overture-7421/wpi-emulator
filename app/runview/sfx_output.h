#pragma once

// Where sound effects come out. The run view asks this interface to play a resource URL; the real implementation uses
// Qt Multimedia, tests use a recording fake so they need no audio device.

#include <QUrl>

#include <memory>

namespace wpiemu::app::runview {

class ISfxOutput {
public:
    virtual ~ISfxOutput() = default;

    // False when there is nothing to play on (no audio output device).
    virtual bool available() const = 0;
    // Loads `url` ahead of time so the first play has no start-up delay.
    virtual void preload(const QUrl& url) = 0;
    // Starts the sound, restarting it when it is already playing. `volume` is 0..1.
    virtual void play(const QUrl& url, double volume) = 0;
    virtual void stopAll() = 0;
};

// The build's real output (Qt Multimedia `QSoundEffect`).
std::unique_ptr<ISfxOutput> createDefaultSfxOutput();

}  // namespace wpiemu::app::runview
