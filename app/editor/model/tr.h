#pragma once

// Translation helper for code that is not a QObject (model / view-model helpers). Use ETR("text");
// run lupdate with `-tr-function-alias translate+=ETR` (context "Editor").

#include <QCoreApplication>
#include <QString>

#define ETR(text) QCoreApplication::translate("Editor", text)
