#include "client/client_settings.h"

namespace wpiemu::app::client {

const char* relayChoiceKey(RelayChoice choice) {
    switch (choice) {
        case RelayChoice::Development: return "development";
        case RelayChoice::Custom: return "custom";
        case RelayChoice::Production: break;
    }
    return "production";
}

RelayChoice relayChoiceFromKey(const std::string& key, RelayChoice fallback) {
    if (key == "production") return RelayChoice::Production;
    if (key == "development") return RelayChoice::Development;
    if (key == "custom") return RelayChoice::Custom;
    return fallback;
}

ClientPrefs ClientSettings::load() const {
    ClientPrefs p;
    const auto text = [&](const char* key, const std::string& fallback) {
        const QVariant v = store_.value(QString::fromLatin1(key));
        return v.isValid() ? v.toString().toStdString() : fallback;
    };
    p.displayName = text(kPrefDisplayName, p.displayName);
    p.directAddress = text(kPrefDirectAddress, p.directAddress);
    p.relay = relayChoiceFromKey(text(kPrefRelay, {}), p.relay);
    p.customRelay = text(kPrefCustomRelay, p.customRelay);
    p.robotPath = text(kPrefRobotPath, p.robotPath);
    const auto number = [&](const char* key, float fallback) {
        bool ok = false;
        const double v = store_.value(QString::fromLatin1(key), fallback).toDouble(&ok);
        return ok ? static_cast<float>(v) : fallback;
    };
    p.drive.deadband = number(kPrefDeadband, p.drive.deadband);
    p.drive.linearScale = number(kPrefLinearScale, p.drive.linearScale);
    p.drive.angularScale = number(kPrefAngularScale, p.drive.angularScale);
    p.drive.fieldRelative = store_.value(QString::fromLatin1(kPrefFieldRelative), p.drive.fieldRelative).toBool();
    p.drive = normalizedDriveConfig(p.drive);
    p.showDiagnostics = store_.value(QString::fromLatin1(kPrefDiagnostics), p.showDiagnostics).toBool();
    return p;
}

void ClientSettings::save(const ClientPrefs& prefs) {
    const auto put = [&](const char* key, const QVariant& value) { store_.setValue(QString::fromLatin1(key), value); };
    const DriveInputConfig drive = normalizedDriveConfig(prefs.drive);
    put(kPrefDisplayName, QString::fromStdString(prefs.displayName));
    put(kPrefDirectAddress, QString::fromStdString(prefs.directAddress));
    put(kPrefRelay, QString::fromLatin1(relayChoiceKey(prefs.relay)));
    put(kPrefCustomRelay, QString::fromStdString(prefs.customRelay));
    put(kPrefRobotPath, QString::fromStdString(prefs.robotPath));
    put(kPrefDeadband, static_cast<double>(drive.deadband));
    put(kPrefLinearScale, static_cast<double>(drive.linearScale));
    put(kPrefAngularScale, static_cast<double>(drive.angularScale));
    put(kPrefFieldRelative, drive.fieldRelative);
    put(kPrefDiagnostics, prefs.showDiagnostics);
    store_.sync();
}

}  // namespace wpiemu::app::client
