#include "common/unit_system.h"

namespace wpiemu::app {

namespace {

using sdk::PropertyUnit;

QString suffixOf(PropertyUnit unit, UnitSystem system) { return QString::fromUtf8(sdk::unitDisplay(unit, system).suffix); }

}  // namespace

QString UnitFormatter::lengthSuffix() const { return suffixOf(PropertyUnit::Meters, system_); }

double UnitFormatter::lengthToDisplay(double meters) const { return sdk::toDisplay(meters, PropertyUnit::Meters, system_); }

double UnitFormatter::lengthFromDisplay(double shown) const { return sdk::fromDisplay(shown, PropertyUnit::Meters, system_); }

QString UnitFormatter::massSuffix() const { return suffixOf(PropertyUnit::Kilograms, system_); }

double UnitFormatter::massToDisplay(double kg) const { return sdk::toDisplay(kg, PropertyUnit::Kilograms, system_); }

double UnitFormatter::massFromDisplay(double shown) const { return sdk::fromDisplay(shown, PropertyUnit::Kilograms, system_); }

QString UnitFormatter::formatLength(double meters, int decimals) const {
    return QString::number(lengthToDisplay(meters), 'f', decimals) + lengthSuffix();
}

QString UnitFormatter::formatMass(double kg, int decimals) const {
    return QString::number(massToDisplay(kg), 'f', decimals) + massSuffix();
}

QString unitSystemKey(UnitSystem system) {
    return system == UnitSystem::Imperial ? QStringLiteral("imperial") : QStringLiteral("metric");
}

UnitSystem unitSystemFromKey(const QString& key, UnitSystem fallback) {
    if (key == QLatin1String("imperial")) return UnitSystem::Imperial;
    if (key == QLatin1String("metric")) return UnitSystem::Metric;
    return fallback;
}

}  // namespace wpiemu::app
