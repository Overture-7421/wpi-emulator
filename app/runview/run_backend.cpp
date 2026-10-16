#include "run_backend.h"

#include <QRegularExpression>

namespace wpiemu::app::runview {

sdk::LogLevel guessLogLevel(const QString& line, LogStream stream) {
    static const QRegularExpression error(QStringLiteral(R"(\b(ERROR|SEVERE|FATAL|Exception|Caused by)\b|^\s+at\s)"));
    static const QRegularExpression warn(QStringLiteral(R"(\b(WARN|WARNING|Warning)\b)"));
    static const QRegularExpression debug(QStringLiteral(R"(\b(DEBUG|FINE)\b)"));
    static const QRegularExpression trace(QStringLiteral(R"(\b(TRACE|FINER|FINEST)\b)"));
    if (error.match(line).hasMatch()) return sdk::LogLevel::Error;
    if (warn.match(line).hasMatch()) return sdk::LogLevel::Warn;
    if (trace.match(line).hasMatch()) return sdk::LogLevel::Trace;
    if (debug.match(line).hasMatch()) return sdk::LogLevel::Debug;
    // Plain stderr output is usually diagnostics, but not necessarily an error.
    return stream == LogStream::Stderr ? sdk::LogLevel::Warn : sdk::LogLevel::Info;
}

void registerRunViewMetaTypes() {
    static const bool done = [] {
        qRegisterMetaType<RunLogEntry>();
        qRegisterMetaType<sdk::proto::DsState>();
        return true;
    }();
    (void)done;
}

}  // namespace wpiemu::app::runview
