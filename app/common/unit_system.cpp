#include "common/unit_system.h"

#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app {

QString UnitFormatter::lengthSuffix() const {
    return system_ == UnitSystem::Imperial ? QStringLiteral(" in") : QStringLiteral(" cm");
}

double UnitFormatter::lengthToDisplay(double meters) const {
    return system_ == UnitSystem::Imperial ? sdk::metersToInches(meters) : meters * 100.0;
}

double UnitFormatter::lengthFromDisplay(double shown) const {
    return system_ == UnitSystem::Imperial ? sdk::inchesToMeters(shown) : shown / 100.0;
}

QString UnitFormatter::massSuffix() const {
    return system_ == UnitSystem::Imperial ? QStringLiteral(" lb") : QStringLiteral(" kg");
}

double UnitFormatter::massToDisplay(double kg) const {
    return system_ == UnitSystem::Imperial ? sdk::kgToPounds(kg) : kg;
}

double UnitFormatter::massFromDisplay(double shown) const {
    return system_ == UnitSystem::Imperial ? sdk::poundsToKg(shown) : shown;
}

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
