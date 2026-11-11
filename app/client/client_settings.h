#pragma once

// What the game client remembers between runs, through the app's key-value port (QSettings in the app, memory in tests). The
// invite is never stored: it is a secret of one hosting session and the host shows a new one every time.

#include <string>

#include "client/input_mapping.h"
#include "ui/key_value_store.h"
#include "ui/relay_endpoints.h"

namespace wpiemu::app::client {

struct ClientPrefs {
    std::string displayName = "Player";
    std::string directAddress;                     // the last host:port typed for a direct connection
    RelayChoice relay = RelayChoice::Production;   // Production unless the player chose otherwise under Advanced
    std::string customRelay;
    std::string robotPath;                         // the last robot.json
    DriveInputConfig drive;                        // deadband and speed scale of the sticks
    bool showDiagnostics = false;                  // the connection overlay (ping, loss, prediction error)

    bool operator==(const ClientPrefs&) const = default;
};

inline constexpr char kPrefDisplayName[] = "client/displayName";
inline constexpr char kPrefDirectAddress[] = "client/directAddress";
inline constexpr char kPrefRelay[] = "client/relay";
inline constexpr char kPrefCustomRelay[] = "client/customRelay";
inline constexpr char kPrefRobotPath[] = "client/robotPath";
inline constexpr char kPrefDeadband[] = "client/drive/deadband";
inline constexpr char kPrefLinearScale[] = "client/drive/linearScale";
inline constexpr char kPrefAngularScale[] = "client/drive/angularScale";
inline constexpr char kPrefFieldRelative[] = "client/drive/fieldRelative";
inline constexpr char kPrefDiagnostics[] = "client/diagnostics";

const char* relayChoiceKey(RelayChoice choice);
RelayChoice relayChoiceFromKey(const std::string& key, RelayChoice fallback = RelayChoice::Production);

class ClientSettings {
public:
    explicit ClientSettings(IKeyValueStore& store) : store_(store) {}

    // Stored values, each brought into range; anything missing or unusable is the default.
    ClientPrefs load() const;
    void save(const ClientPrefs& prefs);

    // The relays this build knows (environment, stored suffix, then the build's): Production and Development.
    RelayPresets relayPresets() const { return app::relayPresets(store_); }

private:
    IKeyValueStore& store_;
};

}  // namespace wpiemu::app::client
