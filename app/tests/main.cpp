#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include <QApplication>
#include <QtGlobal>

int main(int argc, char** argv) {
    // The GUI smoke tests never need a display.
    if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);
    QApplication::setOrganizationName(QStringLiteral("WPI Emulator Tests"));
    QApplication::setApplicationName(QStringLiteral("wpiemu_app_tests"));

    doctest::Context context(argc, argv);
    return context.run();
}
