#include "common/ui_resources.h"

namespace wpiemu::app {

UiResources::UiResources(ThemeManager& theme, QObject* parent) : QObject(parent), theme_(theme) {
    connect(&theme_, &ThemeManager::themeChanged, this, &UiResources::themeChanged);
}

QColor UiResources::color(IconTone tone) const {
    const ThemeTokens& t = theme_.tokens();
    switch (tone) {
        case IconTone::Text: return t.text;
        case IconTone::Muted: return t.mutedText;
        case IconTone::Accent: return t.accent;
        case IconTone::OnAccent: return t.accentText;
        case IconTone::Success: return t.success;
        case IconTone::Warning: return t.warning;
        case IconTone::Error: return t.error;
    }
    return t.text;
}

QIcon UiResources::icon(const QString& name, IconTone tone) const { return icons_.icon(name, color(tone)); }

}  // namespace wpiemu::app
