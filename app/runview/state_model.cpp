#include "state_model.h"

#include <QFont>
#include <QPainter>
#include <QPixmap>

#include <algorithm>
#include <cmath>

#include "scene_parts.h"

namespace wpiemu::app::runview {

namespace {

const QString kDash = QStringLiteral("—");

const sdk::MechanismConfig* findMech(const sdk::RobotConfig* c, const std::string& id) {
    return c != nullptr ? c->findMechanism(id) : nullptr;
}

template <typename T>
const T* findById(const std::vector<T>* items, const std::string& id) {
    if (items == nullptr) return nullptr;
    for (const T& t : *items)
        if (t.id == id) return &t;
    return nullptr;
}

QString labelOf(const std::string& label, const std::string& id) {
    return QString::fromStdString(label.empty() ? id : label);
}

}  // namespace

StateModel::StateModel(const ViewportTheme& theme, QObject* parent) : QAbstractItemModel(parent), theme_(theme) {}

StateModel::~StateModel() = default;

QString StateModel::unitSymbol(sdk::PropertyUnit unit) {
    using U = sdk::PropertyUnit;
    switch (unit) {
        case U::Meters: return QStringLiteral("m");
        case U::MetersPerSecond: return QStringLiteral("m/s");
        case U::Radians: return QStringLiteral("rad");
        case U::RadiansPerSecond: return QStringLiteral("rad/s");
        case U::Kilograms: return QStringLiteral("kg");
        case U::Seconds: return QStringLiteral("s");
        case U::Volts: return QStringLiteral("V");
        case U::Amps: return QStringLiteral("A");
        case U::Newtons: return QStringLiteral("N");
        case U::NewtonMeters: return QStringLiteral("N·m");
        case U::KgM2: return QStringLiteral("kg·m²");
        case U::Percent: return QStringLiteral("%");
        case U::Hertz: return QStringLiteral("Hz");
        case U::None:
        case U::Ratio:
        case U::Count: break;
    }
    return {};
}

QString StateModel::formatNumber(double value, const QString& unit) {
    const double a = std::fabs(value);
    const bool whole = unit.isEmpty() && a < 1e6 && std::fabs(value - std::round(value)) < 1e-9;
    const int decimals = whole ? 0 : (a >= 1000.0 ? 0 : (a >= 100.0 ? 1 : 2));
    QString text = QString::number(value, 'f', decimals);
    if (!unit.isEmpty()) text += QLatin1Char(' ') + unit;
    return text;
}

void StateModel::setScene(const RunSceneInfo& info) {
    info_ = info;
    update(nullptr);
}

StateModel::Node StateModel::build(const sdk::ISimSnapshot* snapshot) const {
    Node root;
    if (snapshot == nullptr) return root;
    const sdk::RobotConfig* cfg = info_.config.get();

    const auto section = [&](const char* key, const QString& title, std::vector<Node> rows) {
        if (rows.empty()) return;
        Node n;
        n.key = QString::fromLatin1(key);
        n.name = title;
        n.value = QString::number(rows.size());
        n.tone = Tone::Muted;
        n.children = std::move(rows);
        root.children.push_back(std::move(n));
    };
    const auto field = [](const QString& parentKey, const QString& name, const QString& value,
                          Tone tone = Tone::Normal, const QString& tip = {}) {
        Node n;
        n.key = parentKey + QLatin1Char('/') + name;
        n.name = name;
        n.value = value;
        n.tone = tone;
        n.tooltip = tip;
        return n;
    };

    std::vector<Node> mechanisms;
    for (const sdk::MechanismState& m : snapshot->mechanisms()) {
        const sdk::MechanismConfig* mc = findMech(cfg, m.id);
        Node n;
        n.key = QStringLiteral("mechanism:") + QString::fromStdString(m.id);
        n.name = labelOf(mc != nullptr ? mc->label : std::string(), m.id);
        n.tooltip = QString::fromStdString(m.type);
        n.hasDot = true;
        n.dot = m.active ? Tone::Good : Tone::Muted;
        n.value = m.active ? tr("active") : tr("idle");
        for (const sdk::NamedValue& v : m.values) {
            const DisplayValue d = toDisplayValue(v);
            n.children.push_back(
                field(n.key, QString::fromStdString(d.name), formatNumber(d.value, QString::fromStdString(d.unit))));
        }
        mechanisms.push_back(std::move(n));
    }
    section("section:mechanisms", tr("Mechanisms"), std::move(mechanisms));

    std::vector<Node> sensors;
    for (const sdk::SensorState& s : snapshot->sensors()) {
        const sdk::SensorConfig* sc = nullptr;
        if (cfg != nullptr)
            for (const sdk::SensorConfig& c : cfg->sensors)
                if (c.id == s.id) sc = &c;
        Node n;
        n.key = QStringLiteral("sensor:") + QString::fromStdString(s.id);
        n.name = labelOf(sc != nullptr ? sc->label : std::string(), s.id);
        const bool distance = sc == nullptr || sc->kind == sdk::SensorKind::Distance;
        n.value = distance ? formatNumber(s.reading, QStringLiteral("m")) : (s.reading >= 0.5 ? tr("true") : tr("false"));
        n.tooltip = QString::fromUtf8(sdk::sensorKindInfo(sc != nullptr ? sc->kind : sdk::SensorKind::Distance).displayName);

        const std::string source = sc != nullptr ? sc->source : std::string();
        const sdk::GameValueInfo* gv = nullptr;
        for (const sdk::GameValueInfo& g : info_.gameValues)
            if (g.id == source) gv = &g;
        const bool unavailable = !s.sourceValue.has_value() && !source.empty();
        if (unavailable) {
            n.tone = Tone::Warning;
            n.tooltip = tr("The game does not publish this sensor's source; it reads idle.");
        }
        Node src = field(n.key, tr("Source"),
                         source.empty() ? tr("none") : (gv != nullptr && !gv->label.empty() ? QString::fromStdString(gv->label)
                                                                                         : QString::fromStdString(source)),
                         Tone::Normal, QString::fromStdString(source));
        n.children.push_back(std::move(src));
        QString input = kDash;
        if (s.sourceValue) {
            const sdk::PropertyUnit u = gv != nullptr ? gv->unit : sdk::PropertyUnit::None;
            const DisplayValue d = toDisplayValue({std::string(), *s.sourceValue, unitSymbol(u).toStdString()});
            input = formatNumber(d.value, QString::fromStdString(d.unit));
        }
        n.children.push_back(field(n.key, tr("Input"), input, unavailable ? Tone::Warning : Tone::Normal));
        sensors.push_back(std::move(n));
    }
    section("section:sensors", tr("Sensors"), std::move(sensors));

    std::vector<Node> zones;
    for (const sdk::ZoneState& z : snapshot->zones()) {
        const sdk::ZoneConfig* zc = findById(cfg != nullptr ? &cfg->zones : nullptr, z.id);
        Node n;
        n.key = QStringLiteral("zone:") + QString::fromStdString(z.id);
        n.name = labelOf(zc != nullptr ? zc->label : std::string(), z.id);
        n.hasDot = true;
        n.dot = z.active ? Tone::Good : Tone::Muted;
        n.value = z.active ? tr("active") : tr("inactive");
        if (zc != nullptr) {
            QString kind = QString::fromStdString(zc->kind);
            for (const sdk::ZoneKindInfo& k : info_.zoneKinds)
                if (k.kind == zc->kind && !k.displayName.empty()) kind = QString::fromStdString(k.displayName);
            n.children.push_back(field(n.key, tr("Kind"), kind));
        }
        zones.push_back(std::move(n));
    }
    section("section:zones", tr("Zones"), std::move(zones));

    std::vector<Node> cameras;
    for (const sdk::CameraState& c : snapshot->cameras()) {
        const sdk::CameraConfig* cc = findById(cfg != nullptr ? &cfg->cameras : nullptr, c.id);
        Node n;
        n.key = QStringLiteral("camera:") + QString::fromStdString(c.id);
        n.name = labelOf(cc != nullptr ? cc->label : std::string(), c.id);
        const std::size_t count = c.visibleTagIds.size();
        n.value = count == 0 ? tr("no tags") : tr("%n tag(s)", nullptr, static_cast<int>(count));
        n.tone = count == 0 ? Tone::Muted : Tone::Good;
        if (cc != nullptr)
            n.children.push_back(field(n.key, tr("Kind"),
                                       cc->kind == sdk::CameraKind::PhotonVision ? QStringLiteral("PhotonVision")
                                                                                 : QStringLiteral("Limelight")));
        QStringList ids;
        for (int id : c.visibleTagIds) ids << QString::number(id);
        n.children.push_back(field(n.key, tr("Visible tags"), ids.isEmpty() ? kDash : ids.join(QStringLiteral(", "))));
        cameras.push_back(std::move(n));
    }
    section("section:cameras", tr("Cameras"), std::move(cameras));
    return root;
}

void StateModel::link(Node& node) {
    int row = 0;
    for (Node& c : node.children) {
        c.parent = &node;
        c.row = row++;
        link(c);
    }
}

bool StateModel::sameShape(const Node& a, const Node& b) {
    if (a.key != b.key || a.hasDot != b.hasDot || a.children.size() != b.children.size()) return false;
    for (std::size_t i = 0; i < a.children.size(); ++i)
        if (!sameShape(a.children[i], b.children[i])) return false;
    return true;
}

void StateModel::merge(Node& cur, Node&& next, const QModelIndex& idx) {
    const bool nameChanged = cur.name != next.name || cur.dot != next.dot;
    const bool valueChanged = cur.value != next.value || cur.tone != next.tone || cur.tooltip != next.tooltip;
    cur.name = std::move(next.name);
    cur.dot = next.dot;
    cur.value = std::move(next.value);
    cur.tone = next.tone;
    cur.tooltip = std::move(next.tooltip);
    if (idx.isValid()) {
        if (nameChanged) emit dataChanged(idx, idx);
        if (valueChanged) {
            const QModelIndex v = idx.siblingAtColumn(1);
            emit dataChanged(v, v);
        }
    }
    for (std::size_t i = 0; i < cur.children.size(); ++i)
        merge(cur.children[i], std::move(next.children[i]), index(static_cast<int>(i), 0, idx));
}

void StateModel::update(const sdk::ISimSnapshot* snapshot) {
    Node next = build(snapshot);
    if (sameShape(root_, next)) {
        for (std::size_t i = 0; i < root_.children.size(); ++i)
            merge(root_.children[i], std::move(next.children[i]), index(static_cast<int>(i), 0, {}));
        return;
    }
    beginResetModel();
    root_ = std::move(next);
    link(root_);
    endResetModel();
}

bool StateModel::isEmpty() const { return root_.children.empty(); }

const StateModel::Node* StateModel::find(const Node& node, const QString& key) {
    if (node.key == key) return &node;
    for (const Node& c : node.children)
        if (const Node* f = find(c, key)) return f;
    return nullptr;
}

QModelIndex StateModel::findKey(const QString& key) const {
    const Node* n = find(root_, key);
    return n != nullptr && n != &root_ ? indexOf(n) : QModelIndex();
}

QModelIndex StateModel::indexOf(const Node* node, int column) const {
    return createIndex(node->row, column, const_cast<Node*>(node));
}

const StateModel::Node* StateModel::nodeAt(const QModelIndex& index) const {
    return index.isValid() ? static_cast<const Node*>(index.internalPointer()) : &root_;
}

int StateModel::rowCount(const QModelIndex& parent) const {
    if (parent.column() > 0) return 0;
    return static_cast<int>(nodeAt(parent)->children.size());
}

QModelIndex StateModel::index(int row, int column, const QModelIndex& parent) const {
    if (column < 0 || column >= 2 || row < 0) return {};
    const Node* p = nodeAt(parent);
    if (row >= static_cast<int>(p->children.size())) return {};
    return createIndex(row, column, const_cast<Node*>(&p->children[static_cast<std::size_t>(row)]));
}

QModelIndex StateModel::parent(const QModelIndex& child) const {
    if (!child.isValid()) return {};
    const Node* p = static_cast<const Node*>(child.internalPointer())->parent;
    if (p == nullptr || p == &root_) return {};
    return indexOf(p);
}

QColor StateModel::color(Tone tone) const {
    switch (tone) {
        case Tone::Good: return theme_.success();
        case Tone::Warning: return theme_.warning();
        case Tone::Muted: return theme_.mutedText();
        case Tone::Normal: break;
    }
    return theme_.text();
}

QVariant StateModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid()) return {};
    const Node& n = *static_cast<const Node*>(index.internalPointer());
    const bool top = n.parent == &root_;
    switch (role) {
        case Qt::DisplayRole: return index.column() == 0 ? n.name : n.value;
        case Qt::ToolTipRole: return n.tooltip.isEmpty() ? QVariant() : QVariant(n.tooltip);
        case Qt::ForegroundRole:
            if (index.column() == 1 && n.tone != Tone::Normal) return color(n.tone);
            return {};
        case Qt::FontRole:
            if (top && index.column() == 0) {
                QFont f;
                f.setBold(true);
                return f;
            }
            return {};
        case Qt::DecorationRole: {
            if (index.column() != 0 || !n.hasDot) return {};
            const QColor c = color(n.dot);
            auto it = dots_.find(c.rgba());
            if (it == dots_.end()) {
                QPixmap pm(QSize(10, 10) * 2);
                pm.setDevicePixelRatio(2.0);
                pm.fill(Qt::transparent);
                QPainter p(&pm);
                p.setRenderHint(QPainter::Antialiasing);
                p.setPen(Qt::NoPen);
                p.setBrush(c);
                p.drawEllipse(QRectF(1, 1, 8, 8));
                p.end();
                it = dots_.insert(c.rgba(), QIcon(pm));
            }
            return it.value();
        }
        case KeyRole: return n.key;
        case ToneRole: return static_cast<int>(n.tone);
        default: return {};
    }
}

QVariant StateModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole) return {};
    return section == 0 ? tr("Name") : tr("Value");
}

}  // namespace wpiemu::app::runview
