// Test entry point for the robot editor. Qt widgets need a QApplication (offscreen platform on CI);
// doctest runs after it exists. Run with QT_QPA_PLATFORM=offscreen (CTest sets it).
#include <QApplication>
#include <QEventLoop>
#include <QTimer>

#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include "test_support.h"

namespace wpiemu::app::editor::testing {

void pumpEvents(int milliseconds) {
    QEventLoop loop;
    QTimer::singleShot(milliseconds, &loop, &QEventLoop::quit);
    loop.exec();
}

}  // namespace wpiemu::app::editor::testing

int main(int argc, char** argv) {
    qputenv("QT_QPA_PLATFORM", qgetenv("QT_QPA_PLATFORM").isEmpty() ? QByteArray("offscreen") : qgetenv("QT_QPA_PLATFORM"));
    QApplication app(argc, argv);
    doctest::Context context(argc, argv);
    return context.run();
}
