#include "joystick_types.h"

#include <QKeySequence>
#include <QVariantList>

#include <algorithm>
#include <cmath>

#include "ui/qt_text.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app::runview::joy {
namespace {

constexpr ui::Tr tr{"Joystick"};

bool keyDown(const Input& in, const RawInput& raw) { return in.kind == Input::Kind::Key && raw.keys.count(in.code) != 0; }

QVariantMap axisToVariant(const AxisBinding& a) {
    return {{QStringLiteral("first"), a.first.toString()},
            {QStringLiteral("second"), a.second.toString()},
            {QStringLiteral("invert"), a.invert},
            {QStringLiteral("deadband"), static_cast<double>(a.deadband)}};
}

AxisBinding axisFromVariant(const QVariantMap& m, const AxisBinding& fallback) {
    AxisBinding a = fallback;
    if (m.contains(QStringLiteral("first"))) a.first = Input::fromString(m.value(QStringLiteral("first")).toString());
    if (m.contains(QStringLiteral("second"))) a.second = Input::fromString(m.value(QStringLiteral("second")).toString());
    a.invert = m.value(QStringLiteral("invert"), a.invert).toBool();
    a.deadband = std::clamp(m.value(QStringLiteral("deadband"), static_cast<double>(a.deadband)).toFloat(), 0.f, 0.9f);
    return a;
}

}  // namespace

QString Input::toString() const {
    switch (kind) {
        case Kind::None: return QStringLiteral("none");
        case Kind::Key: return QStringLiteral("key:%1").arg(code);
        case Kind::PadAxis: return QStringLiteral("axis:%1").arg(code);
        case Kind::PadButton: return QStringLiteral("button:%1").arg(code);
        case Kind::PadPov: return QStringLiteral("pov:%1").arg(code);
    }
    return QStringLiteral("none");
}

Input Input::fromString(const QString& text) {
    const int colon = text.indexOf(QLatin1Char(':'));
    if (colon < 0) return none();
    const QString head = text.left(colon);
    bool ok = false;
    const int code = text.mid(colon + 1).toInt(&ok);
    if (!ok) return none();
    if (head == QLatin1String("key")) return key(code);
    if (head == QLatin1String("axis") && code >= 0) return padAxis(code);
    if (head == QLatin1String("button") && code >= 0) return padButton(code);
    if (head == QLatin1String("pov") && code >= 0) return padPov(code);
    return none();
}

QString Input::displayText() const {
    switch (kind) {
        case Kind::None: return tr("—");
        case Kind::Key: return QKeySequence(code).toString(QKeySequence::NativeText);
        case Kind::PadAxis: return tr("Axis %1").arg(code);
        case Kind::PadButton: return tr("Button %1").arg(code + 1);
        case Kind::PadPov: return tr("D-pad");
    }
    return {};
}

JoystickProfile JoystickProfile::defaultKeyboard() {
    JoystickProfile p;
    p.axes[0] = {Input::key(Qt::Key_A), Input::key(Qt::Key_D), false, 0.f};
    p.axes[1] = {Input::key(Qt::Key_W), Input::key(Qt::Key_S), false, 0.f};
    p.axes[2] = {Input::none(), Input::key(Qt::Key_U), false, 0.f};
    p.axes[3] = {Input::none(), Input::key(Qt::Key_O), false, 0.f};
    p.axes[4] = {Input::key(Qt::Key_Left), Input::key(Qt::Key_Right), false, 0.f};
    p.axes[5] = {Input::key(Qt::Key_Up), Input::key(Qt::Key_Down), false, 0.f};
    const int keys[kButtonCount] = {Qt::Key_Z, Qt::Key_X, Qt::Key_C, Qt::Key_V, Qt::Key_Q, Qt::Key_E,
                                    Qt::Key_R, Qt::Key_F, Qt::Key_G, Qt::Key_H, Qt::Key_N, Qt::Key_M};
    for (int i = 0; i < kButtonCount; ++i) p.buttons[static_cast<std::size_t>(i)] = Input::key(keys[i]);
    p.pov.up = Input::key(Qt::Key_I);
    p.pov.right = Input::key(Qt::Key_L);
    p.pov.down = Input::key(Qt::Key_K);
    p.pov.left = Input::key(Qt::Key_J);
    return p;
}

JoystickProfile JoystickProfile::defaultGamepad() {
    JoystickProfile p;
    for (int i = 0; i < kAxisCount; ++i) p.axes[static_cast<std::size_t>(i)].first = Input::padAxis(i);
    for (int i = 0; i < kButtonCount; ++i) p.buttons[static_cast<std::size_t>(i)] = Input::padButton(i);
    p.pov.pad = Input::padPov(0);
    return p;
}

QVariantMap JoystickProfile::toVariant() const {
    QVariantList ax, bt;
    for (const auto& a : axes) ax << axisToVariant(a);
    for (const auto& b : buttons) bt << b.toString();
    const QVariantMap povMap{{QStringLiteral("up"), pov.up.toString()},     {QStringLiteral("right"), pov.right.toString()},
                             {QStringLiteral("down"), pov.down.toString()}, {QStringLiteral("left"), pov.left.toString()},
                             {QStringLiteral("pad"), pov.pad.toString()}};
    return {{QStringLiteral("axes"), ax}, {QStringLiteral("buttons"), bt}, {QStringLiteral("pov"), povMap}};
}

JoystickProfile JoystickProfile::fromVariant(const QVariantMap& map, const JoystickProfile& fallback) {
    JoystickProfile p = fallback;
    const QVariantList ax = map.value(QStringLiteral("axes")).toList();
    for (int i = 0; i < kAxisCount && i < ax.size(); ++i)
        p.axes[static_cast<std::size_t>(i)] = axisFromVariant(ax[i].toMap(), fallback.axes[static_cast<std::size_t>(i)]);
    const QVariantList bt = map.value(QStringLiteral("buttons")).toList();
    for (int i = 0; i < kButtonCount && i < bt.size(); ++i) p.buttons[static_cast<std::size_t>(i)] = Input::fromString(bt[i].toString());
    const QVariantMap pv = map.value(QStringLiteral("pov")).toMap();
    if (!pv.isEmpty()) {
        p.pov.up = Input::fromString(pv.value(QStringLiteral("up")).toString());
        p.pov.right = Input::fromString(pv.value(QStringLiteral("right")).toString());
        p.pov.down = Input::fromString(pv.value(QStringLiteral("down")).toString());
        p.pov.left = Input::fromString(pv.value(QStringLiteral("left")).toString());
        p.pov.pad = Input::fromString(pv.value(QStringLiteral("pad")).toString());
    }
    return p;
}

float evaluateAxis(const AxisBinding& b, const RawInput& raw) {
    float v = 0.f;
    if (b.first.kind == Input::Kind::PadAxis) {
        const auto idx = static_cast<std::size_t>(std::max(0, b.first.code));
        v = idx < raw.axes.size() ? raw.axes[idx] : 0.f;
        const float mag = std::fabs(v);
        v = mag <= b.deadband ? 0.f : std::copysign((mag - b.deadband) / (1.f - b.deadband), v);
    } else {
        v = (keyDown(b.second, raw) ? 1.f : 0.f) - (keyDown(b.first, raw) ? 1.f : 0.f);
    }
    if (b.invert) v = -v;
    return std::clamp(v, -1.f, 1.f);
}

bool evaluateButton(const Input& in, const RawInput& raw) {
    switch (in.kind) {
        case Input::Kind::Key: return raw.keys.count(in.code) != 0;
        case Input::Kind::PadButton: {
            const auto idx = static_cast<std::size_t>(std::max(0, in.code));
            return idx < raw.buttons.size() && raw.buttons[idx] != 0;
        }
        default: return false;
    }
}

std::int16_t evaluatePov(const PovBinding& b, const RawInput& raw) {
    if (b.pad.kind == Input::Kind::PadPov) {
        const auto idx = static_cast<std::size_t>(std::max(0, b.pad.code));
        if (idx < raw.povs.size() && raw.povs[idx] >= 0) return raw.povs[idx];
    }
    const int dx = (keyDown(b.right, raw) ? 1 : 0) - (keyDown(b.left, raw) ? 1 : 0);
    const int dy = (keyDown(b.up, raw) ? 1 : 0) - (keyDown(b.down, raw) ? 1 : 0);
    if (dx == 0 && dy == 0) return -1;
    const double deg = sdk::radToDeg(std::atan2(static_cast<double>(dx), static_cast<double>(dy)));
    return static_cast<std::int16_t>(static_cast<int>(std::lround(deg < 0 ? deg + 360.0 : deg)) % 360);
}

sdk::proto::JoystickState evaluate(const JoystickProfile& profile, const RawInput& raw, bool present) {
    sdk::proto::JoystickState s;
    s.present = present;
    if (!present) return s;
    s.axes.reserve(kAxisCount);
    for (const auto& a : profile.axes) s.axes.push_back(evaluateAxis(a, raw));
    for (int i = 0; i < kButtonCount; ++i)
        if (evaluateButton(profile.buttons[static_cast<std::size_t>(i)], raw)) s.buttons |= (1u << i);
    s.buttonCount = static_cast<std::uint8_t>(kButtonCount);
    s.povs.push_back(evaluatePov(profile.pov, raw));
    return s;
}

QString axisName(int axis) {
    switch (axis) {
        case 0: return tr("Left stick X");
        case 1: return tr("Left stick Y");
        case 2: return tr("Left trigger");
        case 3: return tr("Right trigger");
        case 4: return tr("Right stick X");
        case 5: return tr("Right stick Y");
        default: return tr("Axis %1").arg(axis);
    }
}

QString buttonName(int button) {
    static const char* const names[] = {
        QT_TRANSLATE_NOOP("Joystick", "A"),           QT_TRANSLATE_NOOP("Joystick", "B"),
        QT_TRANSLATE_NOOP("Joystick", "X"),           QT_TRANSLATE_NOOP("Joystick", "Y"),
        QT_TRANSLATE_NOOP("Joystick", "Left bumper"), QT_TRANSLATE_NOOP("Joystick", "Right bumper"),
        QT_TRANSLATE_NOOP("Joystick", "Back"),        QT_TRANSLATE_NOOP("Joystick", "Start"),
        QT_TRANSLATE_NOOP("Joystick", "Left stick"),  QT_TRANSLATE_NOOP("Joystick", "Right stick")};
    if (button >= 0 && button < 10) return QStringLiteral("%1 · %2").arg(button + 1).arg(tr(names[button]));
    return tr("Button %1").arg(button + 1);
}

}  // namespace wpiemu::app::runview::joy
