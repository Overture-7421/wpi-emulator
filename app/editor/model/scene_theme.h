#pragma once

// Colours of the 3D scene. Derived from the widget palette so the viewport follows the light/dark
// theme of the app; computed in C++ and handed to QML (no colours are hard-coded in the QML).

#include <QColor>
#include <QPalette>
#include <QString>

namespace wpiemu::app::editor {

struct SceneTheme {
    QColor background{"#1e2127"};
    QColor gridMinor{"#2c313a"};
    QColor gridMajor{"#3b424f"};
    QColor floor{"#23272e"};
    QColor hull{"#6f86a3"};
    QColor wheel{"#4b525c"};
    QColor shape{"#8f98a5"};
    QColor mechanism{"#d9a441"};
    QColor accent{"#4da3ff"};      // selection
    QColor com{"#ff9f1c"};
    QColor link{"#2ec4b6"};
    QColor limelight{"#9be15d"};
    QColor photonVision{"#c77dff"};
    QColor axisX{"#e5484d"};
    QColor axisY{"#46a758"};
    QColor axisZ{"#3e63dd"};
    QColor text{"#d7dae0"};
    bool isDark = true;

    static SceneTheme dark() { return SceneTheme{}; }
    static SceneTheme light() {
        SceneTheme t;
        t.isDark = false;
        t.background = QColor("#eef1f5");
        t.gridMinor = QColor("#d9dee6");
        t.gridMajor = QColor("#b9c1cd");
        t.floor = QColor("#e3e7ee");
        t.hull = QColor("#5a7aa3");
        t.wheel = QColor("#3c424b");
        t.shape = QColor("#7d8794");
        t.mechanism = QColor("#e0a020");
        t.accent = QColor("#0a6cff");
        t.link = QColor("#12a595");
        t.limelight = QColor("#4f9d1e");
        t.photonVision = QColor("#8a3fd1");
        t.text = QColor("#2a2f38");
        return t;
    }
    static SceneTheme fromPalette(const QPalette& p) { return p.color(QPalette::Window).lightness() < 128 ? dark() : light(); }
};

}  // namespace wpiemu::app::editor
