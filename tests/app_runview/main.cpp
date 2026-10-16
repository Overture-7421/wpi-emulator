// Entry point of the run view test executable: one QApplication (offscreen in CI) shared by all cases.
#include <QApplication>

#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

int main(int argc, char** argv) {
    qputenv("QT_QPA_PLATFORM", qgetenv("QT_QPA_PLATFORM").isEmpty() ? QByteArray("offscreen") : qgetenv("QT_QPA_PLATFORM"));
    QApplication app(argc, argv);
    QApplication::setOrganizationName(QStringLiteral("wpiemu-tests"));
    QApplication::setApplicationName(QStringLiteral("runview-tests"));

    doctest::Context context(argc, argv);
    return context.run();
}
