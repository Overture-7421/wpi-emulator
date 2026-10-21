#pragma once

// Translation helper for code that is not a QObject (model / view-model helpers). Use ETR("text") for the "Editor"
// context; run lupdate with `-tr-function-alias translate+=ETR`.

#include "ui/qt_text.h"

inline constexpr wpiemu::app::ui::Tr ETR{"Editor"};
