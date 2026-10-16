#include "common/theme.h"

#include <QApplication>
#include <QGuiApplication>
#include <QStyleHints>

namespace wpiemu::app {

ThemeTokens lightTokens() {
    ThemeTokens t;
    t.dark = false;
    t.window = QColor("#f3f5f9");
    t.base = QColor("#ffffff");
    t.altBase = QColor("#eef1f6");
    t.card = QColor("#ffffff");
    t.text = QColor("#1b2330");
    t.mutedText = QColor("#5d6a7b");
    t.border = QColor("#d6dce5");
    t.hover = QColor("#e8edf5");
    t.accent = QColor("#2f6df6");
    t.accentHover = QColor("#1f57d6");
    t.accentText = QColor("#ffffff");
    t.success = QColor("#17995a");
    t.warning = QColor("#b86e00");
    t.error = QColor("#d03b3b");
    return t;
}

ThemeTokens darkTokens() {
    ThemeTokens t;
    t.dark = true;
    t.window = QColor("#14181f");
    t.base = QColor("#1b212b");
    t.altBase = QColor("#202733");
    t.card = QColor("#1b212b");
    t.text = QColor("#e7ebf1");
    t.mutedText = QColor("#9aa6b6");
    t.border = QColor("#2f3746");
    t.hover = QColor("#283040");
    t.accent = QColor("#5b8cff");
    t.accentHover = QColor("#7ca2ff");
    t.accentText = QColor("#0b1020");
    t.success = QColor("#3ecf8e");
    t.warning = QColor("#f0b429");
    t.error = QColor("#ff7070");
    return t;
}

QPalette makePalette(const ThemeTokens& t) {
    QPalette p;
    p.setColor(QPalette::Window, t.window);
    p.setColor(QPalette::WindowText, t.text);
    p.setColor(QPalette::Base, t.base);
    p.setColor(QPalette::AlternateBase, t.altBase);
    p.setColor(QPalette::ToolTipBase, t.card);
    p.setColor(QPalette::ToolTipText, t.text);
    p.setColor(QPalette::Text, t.text);
    p.setColor(QPalette::PlaceholderText, t.mutedText);
    p.setColor(QPalette::Button, t.card);
    p.setColor(QPalette::ButtonText, t.text);
    p.setColor(QPalette::BrightText, t.error);
    p.setColor(QPalette::Highlight, t.accent);
    p.setColor(QPalette::HighlightedText, t.accentText);
    p.setColor(QPalette::Link, t.accent);
    p.setColor(QPalette::Mid, t.border);
    p.setColor(QPalette::Midlight, t.hover);
    p.setColor(QPalette::Dark, t.border);
    p.setColor(QPalette::Light, t.card);
    p.setColor(QPalette::Disabled, QPalette::Text, t.mutedText);
    p.setColor(QPalette::Disabled, QPalette::ButtonText, t.mutedText);
    p.setColor(QPalette::Disabled, QPalette::WindowText, t.mutedText);
    return p;
}

QString buildStyleSheet(const ThemeTokens& t) {
    const QString sheet = QStringLiteral(R"(
QWidget { font-size: 13px; }
QMainWindow, QDialog { background: %window%; }
QToolTip { background: %card%; color: %text%; border: 1px solid %border%; padding: 4px 6px; }

QLabel[role="title"] { font-size: 22px; font-weight: 600; }
QLabel[role="subtitle"] { color: %muted%; font-size: 13px; }
QLabel[role="section"] { font-size: 14px; font-weight: 600; }
QLabel[role="muted"] { color: %muted%; }
QLabel[role="error"] { color: %error%; }
QLabel[role="warning"] { color: %warning%; }
QLabel[role="success"] { color: %success%; }

QFrame[card="true"] { background: %card%; border: 1px solid %border%; border-radius: 10px; }
QFrame[card="true"] QLabel { background: transparent; }
QFrame[dropZone="true"] { background: %alt%; border: 2px dashed %border%; border-radius: 12px; }
QFrame[dropZone="true"][dragActive="true"] { border-color: %accent%; background: %hover%; }
QFrame[banner="warning"] { background: %alt%; border: 1px solid %warning%; border-radius: 8px; }
QFrame[banner="error"] { background: %alt%; border: 1px solid %error%; border-radius: 8px; }
QFrame[banner="success"] { background: %alt%; border: 1px solid %success%; border-radius: 8px; }
QFrame[banner="info"] { background: %alt%; border: 1px solid %accent%; border-radius: 8px; }
QFrame[banner] QLabel { background: transparent; }

QPushButton { background: %card%; color: %text%; border: 1px solid %border%; border-radius: 7px; padding: 7px 16px; }
QPushButton:hover { background: %hover%; }
QPushButton:pressed { background: %border%; }
QPushButton:disabled { color: %muted%; background: %alt%; }
QPushButton:focus { border-color: %accent%; }
QPushButton[primary="true"] { background: %accent%; color: %accentText%; border: 1px solid %accent%; font-weight: 600; }
QPushButton[primary="true"]:hover { background: %accentHover%; border-color: %accentHover%; }
QPushButton[primary="true"]:disabled { background: %border%; color: %muted%; border-color: %border%; }
QPushButton[flat="true"] { background: transparent; border: none; color: %accent%; padding: 4px 8px; }
QPushButton[flat="true"]:hover { background: %hover%; }
QPushButton[stepButton="true"] { color: %text%; text-align: left; padding: 9px 10px; font-size: 14px; }
QPushButton[stepButton="true"]:disabled { color: %muted%; }
QPushButton[stepButton="true"][current="true"] { color: %accent%; font-weight: 600; }

QToolButton { background: transparent; border: 1px solid transparent; border-radius: 7px; padding: 5px 9px; color: %text%; }
QToolButton:hover { background: %hover%; }
QToolButton:checked { background: %accent%; color: %accentText%; }
QToolButton:disabled { color: %muted%; }
QToolButton[segment="true"] { border: 1px solid %border%; border-radius: 0; padding: 6px 16px; background: %card%; }
QToolButton[segment="true"]:hover { background: %hover%; }
QToolButton[segment="true"]:checked { background: %accent%; border-color: %accent%; color: %accentText%; }
QToolButton[segment="true"]:disabled { color: %muted%; background: %alt%; }
QToolButton[segment="true"]:checked:disabled { background: %border%; border-color: %border%; color: %muted%; }
QToolButton[segment="true"][segmentPos="first"] { border-top-left-radius: 7px; border-bottom-left-radius: 7px; }
QToolButton[segment="true"][segmentPos="last"] { border-top-right-radius: 7px; border-bottom-right-radius: 7px; border-left-width: 0; }
QToolButton[segment="true"][segmentPos="middle"] { border-left-width: 0; }

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QPlainTextEdit, QTextEdit {
    background: %base%; color: %text%; border: 1px solid %border%; border-radius: 6px; padding: 5px 8px;
    selection-background-color: %accent%; selection-color: %accentText%; }
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus { border-color: %accent%; }
QLineEdit[invalid="true"], QDoubleSpinBox[invalid="true"] { border-color: %error%; }
QComboBox::drop-down { border: none; width: 26px; }
QComboBox::down-arrow { image: url(%arrowDown%); width: 12px; height: 12px; }
QAbstractSpinBox { padding-right: 24px; }
QAbstractSpinBox::up-button { subcontrol-origin: border; subcontrol-position: top right; width: 22px; border: none; border-left: 1px solid %border%; border-top-right-radius: 6px; }
QAbstractSpinBox::down-button { subcontrol-origin: border; subcontrol-position: bottom right; width: 22px; border: none; border-left: 1px solid %border%; border-bottom-right-radius: 6px; }
QAbstractSpinBox::up-button:hover, QAbstractSpinBox::down-button:hover { background: %hover%; }
QAbstractSpinBox::up-arrow { image: url(%arrowUp%); width: 10px; height: 10px; }
QAbstractSpinBox::down-arrow { image: url(%arrowDown%); width: 10px; height: 10px; }
QComboBox QAbstractItemView { background: %base%; border: 1px solid %border%; selection-background-color: %accent%; selection-color: %accentText%; outline: 0; }
QCheckBox, QRadioButton { spacing: 8px; }

QListWidget, QListView, QTreeView, QTableView { background: %base%; border: 1px solid %border%; border-radius: 8px; outline: 0; }
QListWidget::item { padding: 8px 10px; border-radius: 6px; }
QListWidget::item:hover { background: %hover%; }
QListWidget::item:selected { background: %accent%; color: %accentText%; }

QMenuBar { background: %window%; }
QMenuBar::item { padding: 5px 10px; background: transparent; border-radius: 5px; }
QMenuBar::item:selected { background: %hover%; }
QMenu { background: %card%; border: 1px solid %border%; padding: 5px; }
QMenu::item { padding: 6px 24px 6px 12px; border-radius: 5px; }
QMenu::item:selected { background: %accent%; color: %accentText%; }
QMenu::separator { height: 1px; background: %border%; margin: 5px 8px; }

QToolBar { background: %window%; border: none; border-bottom: 1px solid %border%; padding: 6px 8px; spacing: 6px; }
QToolBar::separator { width: 1px; background: %border%; margin: 4px 6px; }
QStatusBar { background: %window%; border-top: 1px solid %border%; }
QStatusBar::item { border: none; }
QStatusBar QLabel { color: %muted%; padding: 0 6px; }

QTabWidget::pane { border: 1px solid %border%; border-radius: 8px; top: -1px; background: %card%; }
QTabBar::tab { padding: 8px 16px; border: none; color: %muted%; }
QTabBar::tab:selected { color: %text%; border-bottom: 2px solid %accent%; }
QTabBar::tab:hover { color: %text%; }

QProgressBar { background: %alt%; border: none; border-radius: 3px; height: 6px; text-align: center; }
QProgressBar::chunk { background: %accent%; border-radius: 3px; }
QScrollArea { border: none; background: transparent; }
QScrollBar:vertical { background: transparent; width: 10px; margin: 2px; }
QScrollBar::handle:vertical { background: %border%; border-radius: 4px; min-height: 28px; }
QScrollBar::handle:vertical:hover { background: %muted%; }
QScrollBar::add-line, QScrollBar::sub-line { height: 0; width: 0; }
QScrollBar:horizontal { background: transparent; height: 10px; margin: 2px; }
QScrollBar::handle:horizontal { background: %border%; border-radius: 4px; min-width: 28px; }

#WizardSidebar { background: %alt%; border-right: 1px solid %border%; }
#WizardSidebar QLabel { background: transparent; }
#WizardFooter { background: %window%; border-top: 1px solid %border%; }
QLabel[chip="true"] { background: %alt%; border: 1px solid %border%; border-radius: 9px; padding: 1px 9px; color: %muted%; }
QLabel[chip="success"] { background: %alt%; border: 1px solid %success%; border-radius: 9px; padding: 1px 9px; color: %success%; }
QLabel[chip="warning"] { background: %alt%; border: 1px solid %warning%; border-radius: 9px; padding: 1px 9px; color: %warning%; }
)");
    const auto hex = [](const QColor& c) { return c.name(QColor::HexRgb); };
    QString out = sheet;
    out.replace(QLatin1String("%window%"), hex(t.window));
    out.replace(QLatin1String("%base%"), hex(t.base));
    out.replace(QLatin1String("%alt%"), hex(t.altBase));
    out.replace(QLatin1String("%card%"), hex(t.card));
    out.replace(QLatin1String("%text%"), hex(t.text));
    out.replace(QLatin1String("%muted%"), hex(t.mutedText));
    out.replace(QLatin1String("%border%"), hex(t.border));
    out.replace(QLatin1String("%hover%"), hex(t.hover));
    out.replace(QLatin1String("%accentHover%"), hex(t.accentHover));
    out.replace(QLatin1String("%accentText%"), hex(t.accentText));
    out.replace(QLatin1String("%accent%"), hex(t.accent));
    out.replace(QLatin1String("%success%"), hex(t.success));
    out.replace(QLatin1String("%warning%"), hex(t.warning));
    out.replace(QLatin1String("%error%"), hex(t.error));
    // Spin-box and combo arrows: fixed-colour SVGs matching mutedText of each theme (see lightTokens/darkTokens).
    const QString variant = t.dark ? QStringLiteral("dark") : QStringLiteral("light");
    out.replace(QLatin1String("%arrowUp%"), QStringLiteral(":/wpiemu/icons/chevron-up-%1.svg").arg(variant));
    out.replace(QLatin1String("%arrowDown%"), QStringLiteral(":/wpiemu/icons/chevron-down-%1.svg").arg(variant));
    return out;
}

ThemeManager::ThemeManager(QApplication& app, AppSettings& settings, QObject* parent)
    : QObject(parent), app_(app), settings_(settings) {
    app_.setStyle(QStringLiteral("Fusion"));
    connect(&settings_, &AppSettings::changed, this, [this] {
        if (settings_.data().theme != appliedMode_) apply();   // unrelated preferences must not re-polish every widget
    });
    connect(QGuiApplication::styleHints(), &QStyleHints::colorSchemeChanged, this, [this] {
        if (settings_.data().theme == ThemeMode::System) apply();
    });
    apply();
}

bool ThemeManager::resolveDark() const {
    switch (settings_.data().theme) {
        case ThemeMode::Light: return false;
        case ThemeMode::Dark: return true;
        case ThemeMode::System: break;
    }
    return QGuiApplication::styleHints()->colorScheme() == Qt::ColorScheme::Dark;
}

void ThemeManager::apply() {
    // Keep window chrome in step with the choice (Qt >= 6.8); Unknown hands control back to the OS.
    QGuiApplication::styleHints()->setColorScheme(settings_.data().theme == ThemeMode::Light  ? Qt::ColorScheme::Light
                                                  : settings_.data().theme == ThemeMode::Dark ? Qt::ColorScheme::Dark
                                                                                              : Qt::ColorScheme::Unknown);
    tokens_ = resolveDark() ? darkTokens() : lightTokens();
    appliedMode_ = settings_.data().theme;
    app_.setPalette(makePalette(tokens_));
    app_.setStyleSheet(buildStyleSheet(tokens_));
    emit themeChanged();
}

}  // namespace wpiemu::app
