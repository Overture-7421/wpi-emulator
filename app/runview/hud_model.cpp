#include "hud_model.h"

#include <QCoreApplication>

#include <algorithm>
#include <cmath>

namespace wpiemu::app::runview {
namespace {

QString tr(const char* s) { return QCoreApplication::translate("Hud", s); }

QString formatNumber(double v) {
    if (std::fabs(v - std::round(v)) < 1e-9) return QString::number(static_cast<qlonglong>(std::llround(v)));
    return QString::number(v, 'f', std::fabs(v) < 10.0 ? 2 : 1);
}

bool numeric(const sdk::Value& v, double& out) {
    if (v.isNumber()) { out = v.asDouble(); return true; }
    if (v.isBool()) { out = v.asBool() ? 1.0 : 0.0; return true; }
    return false;
}

bool badgeOn(const sdk::Value& v) {
    if (v.isBool()) return v.asBool();
    if (v.isNumber()) return v.asDouble() != 0.0;
    if (v.isString()) {
        const QString s = QString::fromStdString(v.asString()).trimmed().toLower();
        return !(s.isEmpty() || s == QLatin1String("inactive") || s == QLatin1String("off") || s == QLatin1String("false") ||
                 s == QLatin1String("no"));
    }
    return false;
}

QString withUnit(const QString& text, const std::string& unit) {
    return unit.empty() ? text : text + QLatin1Char(' ') + QString::fromStdString(unit);
}

}  // namespace

QString formatClock(double seconds) {
    const int total = static_cast<int>(std::floor(std::max(0.0, seconds)));
    return QStringLiteral("%1:%2").arg(total / 60).arg(total % 60, 2, 10, QLatin1Char('0'));
}

HudItemState evaluateHud(const sdk::HudWidgetDescriptor& d, const sdk::ISimSnapshot* snapshot) {
    HudItemState s;
    s.id = QString::fromStdString(d.id);
    s.title = QString::fromStdString(d.title);
    s.kind = d.kind;
    s.anchor = d.anchor;
    s.order = d.order;
    s.color = QString::fromStdString(d.color);
    s.text = QStringLiteral("—");

    const sdk::Value* v = snapshot != nullptr ? snapshot->hudValue(d.valueKey) : nullptr;
    if (v == nullptr || v->isNull()) return s;
    s.valid = true;

    double number = 0.0;
    switch (d.kind) {
        case sdk::HudWidgetKind::Counter:
            if (numeric(*v, number)) s.text = withUnit(formatNumber(number), d.unit);
            else s.text = QString::fromStdString(v->asString());
            break;
        case sdk::HudWidgetKind::Meter:
        case sdk::HudWidgetKind::Gauge: {
            if (!numeric(*v, number)) { s.valid = false; break; }
            double maxV = d.maxValue;
            if (!d.maxKey.empty() && snapshot != nullptr) {
                double dynamicMax = 0.0;
                if (const sdk::Value* m = snapshot->hudValue(d.maxKey); m != nullptr && numeric(*m, dynamicMax)) maxV = dynamicMax;
            }
            const double span = maxV - d.minValue;
            s.fraction = span > 0.0 ? std::clamp((number - d.minValue) / span, 0.0, 1.0) : 0.0;
            s.text = d.kind == sdk::HudWidgetKind::Meter
                         ? withUnit(QStringLiteral("%1 / %2").arg(formatNumber(number), formatNumber(maxV)), d.unit)
                         : withUnit(formatNumber(number), d.unit);
            break;
        }
        case sdk::HudWidgetKind::Text:
            s.text = v->isString() ? QString::fromStdString(v->asString())
                                   : (numeric(*v, number) ? formatNumber(number) : QString());
            break;
        case sdk::HudWidgetKind::Badge:
            s.active = badgeOn(*v);
            s.text = v->isString() ? QString::fromStdString(v->asString()) : (s.active ? tr("Active") : tr("Inactive"));
            break;
        case sdk::HudWidgetKind::Timer:
            if (numeric(*v, number)) s.text = formatClock(number);
            else s.valid = false;
            break;
    }
    if (!s.valid) s.text = QStringLiteral("—");
    return s;
}

HudModel::HudModel(QObject* parent) : QAbstractListModel(parent) {
    for (int a = 0; a < kAnchorCount; ++a) {
        auto* proxy = new QSortFilterProxyModel(this);
        proxy->setSourceModel(this);
        proxy->setFilterRole(AnchorRole);
        proxy->setFilterFixedString(QString::number(a));
        proxies_.push_back(proxy);
    }
}

void HudModel::setDescriptors(std::vector<sdk::HudWidgetDescriptor> descriptors) {
    std::stable_sort(descriptors.begin(), descriptors.end(), [](const auto& a, const auto& b) {
        return a.anchor != b.anchor ? static_cast<int>(a.anchor) < static_cast<int>(b.anchor) : a.order < b.order;
    });
    beginResetModel();
    descriptors_ = std::move(descriptors);
    states_.clear();
    for (const auto& d : descriptors_) states_.push_back(evaluateHud(d, nullptr));
    endResetModel();
}

void HudModel::update(const sdk::ISimSnapshot* snapshot) {
    for (std::size_t i = 0; i < descriptors_.size(); ++i) {
        HudItemState next = evaluateHud(descriptors_[i], snapshot);
        if (next == states_[i]) continue;
        states_[i] = std::move(next);
        const QModelIndex idx = index(static_cast<int>(i));
        emit dataChanged(idx, idx);
    }
}

int HudModel::rowCount(const QModelIndex& parent) const {
    return parent.isValid() ? 0 : static_cast<int>(states_.size());
}

QVariant HudModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= rowCount()) return {};
    const HudItemState& s = states_[static_cast<std::size_t>(index.row())];
    switch (role) {
        case IdRole: return s.id;
        case TitleRole: return s.title;
        case KindRole: return static_cast<int>(s.kind);
        case AnchorRole: return static_cast<int>(s.anchor);
        case OrderRole: return s.order;
        case ValidRole: return s.valid;
        case ActiveRole: return s.active;
        case FractionRole: return s.fraction;
        case TextRole: return s.text;
        case ColorRole: return s.color;
        default: return {};
    }
}

QHash<int, QByteArray> HudModel::roleNames() const {
    return {{IdRole, "widgetId"}, {TitleRole, "title"},   {KindRole, "kind"},         {AnchorRole, "anchor"},
            {OrderRole, "order"}, {ValidRole, "valid"},   {ActiveRole, "active"},     {FractionRole, "fraction"},
            {TextRole, "text"},   {ColorRole, "accent"}};
}

QObject* HudModel::anchorModel(int anchor) const {
    return anchor >= 0 && anchor < kAnchorCount ? proxies_[static_cast<std::size_t>(anchor)] : nullptr;
}

}  // namespace wpiemu::app::runview
