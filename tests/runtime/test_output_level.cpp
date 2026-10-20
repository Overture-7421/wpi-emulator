#include <doctest/doctest.h>

#include "wpiemu/core/launcher/output_level.h"

using namespace wpiemu::core::launcher;
using wpiemu::sdk::LogLevel;

TEST_CASE("robot output lines are classified by their WPILib and Java markers") {
    CHECK(classifyOutputLine(OutputStream::Stdout, "ERROR 1 something") == LogLevel::Error);
    CHECK(classifyOutputLine(OutputStream::Stdout, "Exception in thread \"main\" java.lang.NullPointerException") == LogLevel::Error);
    CHECK(classifyOutputLine(OutputStream::Stdout, "Caused by: java.io.IOException") == LogLevel::Error);
    CHECK(classifyOutputLine(OutputStream::Stdout, "Warning at frc.robot.Robot: x") == LogLevel::Warn);
    CHECK(classifyOutputLine(OutputStream::Stdout, "WARN: battery") == LogLevel::Warn);
    CHECK(classifyOutputLine(OutputStream::Stdout, "DEBUG x") == LogLevel::Debug);
    CHECK(classifyOutputLine(OutputStream::Stdout, "FINE detail") == LogLevel::Debug);
    CHECK(classifyOutputLine(OutputStream::Stdout, "TRACE x") == LogLevel::Trace);
}

TEST_CASE("robot output: stack frames are errors and the most severe marker wins") {
    CHECK(classifyOutputLine(OutputStream::Stderr, "    at frc.robot.Robot.run(Robot.java:3)") == LogLevel::Error);
    CHECK(classifyOutputLine(OutputStream::Stdout, "\tat frc.robot.Robot.run(Robot.java:3)") == LogLevel::Error);
    CHECK(classifyOutputLine(OutputStream::Stdout, "at home") == LogLevel::Info);          // no leading whitespace: not a frame
    CHECK(classifyOutputLine(OutputStream::Stdout, "  atlas shrugged") == LogLevel::Info);  // "at" must be a whole word
    CHECK(classifyOutputLine(OutputStream::Stdout, "ERROR then Warning then DEBUG") == LogLevel::Error);
    CHECK(classifyOutputLine(OutputStream::Stdout, "Warning and DEBUG") == LogLevel::Warn);
}

TEST_CASE("robot output: markers match whole words only and plain lines default by stream") {
    CHECK(classifyOutputLine(OutputStream::Stdout, "ERRORS are not a marker") == LogLevel::Info);
    CHECK(classifyOutputLine(OutputStream::Stdout, "NullPointerException") == LogLevel::Info);
    CHECK(classifyOutputLine(OutputStream::Stdout, "TERROR_1 and WARNINGS") == LogLevel::Info);
    CHECK(classifyOutputLine(OutputStream::Stdout, "hello") == LogLevel::Info);
    CHECK(classifyOutputLine(OutputStream::Stderr, "hello") == LogLevel::Warn);
    CHECK(classifyOutputLine(OutputStream::Stdout, "") == LogLevel::Info);
    CHECK(classifyOutputLine(OutputStream::Stderr, "[ERROR] build failed") == LogLevel::Error);   // punctuation is a boundary
}
