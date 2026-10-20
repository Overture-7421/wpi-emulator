#pragma once

#include <QString>

#include "wpiemu/sdk/config/unit_display.h"

namespace wpiemu::app {

// How the UI shows lengths and masses: the SDK's unit table (sdk/config/unit_display.h). Persisted values are
// always SI (see sdk/core/units.h).
using sdk::UnitSystem;

class UnitFormatter {
public:
    explicit UnitFormatter(UnitSystem system = UnitSystem::Imperial) : system_(system) {}

    UnitSystem system() const { return system_; }

    // Frame/wheel dimensions: inches or centimetres.
    QString lengthSuffix() const;
    double lengthToDisplay(double meters) const;
    double lengthFromDisplay(double shown) const;

    // Mass: pounds or kilograms.
    QString massSuffix() const;
    double massToDisplay(double kg) const;
    double massFromDisplay(double shown) const;

    QString formatLength(double meters, int decimals = 1) const;
    QString formatMass(double kg, int decimals = 1) const;

private:
    UnitSystem system_;
};

QString unitSystemKey(UnitSystem system);
UnitSystem unitSystemFromKey(const QString& key, UnitSystem fallback = UnitSystem::Imperial);

}  // namespace wpiemu::app
