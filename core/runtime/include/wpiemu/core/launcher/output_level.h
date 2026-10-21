#pragma once

#include <string_view>

#include "wpiemu/core/launcher/process.h"
#include "wpiemu/sdk/core/logger.h"

namespace wpiemu::core::launcher {

// Best-effort level for one line of robot output. WPILib and Java print "ERROR  1 ...", "Warning at ...", stack traces
// ("    at ...", "Caused by: ...") and JUL-style DEBUG/TRACE prefixes; the first match wins in the order Error, Warn,
// Trace, Debug. Anything else is Info, except plain stderr, which is usually diagnostics and defaults to Warn.
sdk::LogLevel classifyOutputLine(OutputStream stream, std::string_view line);

}  // namespace wpiemu::core::launcher
