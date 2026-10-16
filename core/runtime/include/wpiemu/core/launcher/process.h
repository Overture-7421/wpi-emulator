#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/result.h"

namespace wpiemu::core::launcher {

// What to run. `program` is an executable path or a name looked up on PATH (the effective PATH: `env` override, else ours).
struct ProcessSpec {
    std::string program;
    std::vector<std::string> args;
    std::filesystem::path workingDir;            // empty = inherit
    std::map<std::string, std::string> env;      // added to / overriding the inherited environment
    std::vector<std::string> unsetEnv;           // removed from the inherited environment
    std::string label;                           // for logs ("gradlew simulateJava")
};

enum class OutputStream { Stdout, Stderr };

// Receives the process output one line at a time (no trailing newline; "\r\n" and "\n" both end a line; a final
// unterminated line is delivered at EOF). Called from reader threads: implementations must be thread-safe and fast.
class ILogSink {
    WPIEMU_INTERFACE(ILogSink)
public:
    virtual void line(OutputStream stream, std::string_view text) = 0;
};

struct ExitInfo {
    int exitCode = 0;          // the process' own exit code; 128 + signal on POSIX when signalled; -1 if unknown
    bool killedByUs = false;   // kill() / the destructor ended it
    bool signalled = false;    // POSIX: terminated by a signal
};

// One supervised child process tree.
//  * start() never blocks on the child; errors (program not found, cannot spawn) are returned.
//  * The WHOLE TREE is tied to the object: kill() ends every descendant (Windows job object / POSIX process group), and so
//    does the destructor, so a crashed or closed emulator does not leave a robot behind (best effort on a hard crash).
//  * stdout and stderr are captured line by line into the sink given to start(), which must outlive the process.
class IRobotProcess {
    WPIEMU_INTERFACE(IRobotProcess)
public:
    virtual sdk::Status start(const ProcessSpec& spec, ILogSink& sink) = 0;
    virtual bool running() const = 0;
    virtual std::int64_t pid() const = 0;  // 0 before start()
    virtual std::optional<ExitInfo> exitInfo() const = 0;  // set once the process ended
    // True when the process has exited (also true if it was never started).
    virtual bool waitForExit(std::chrono::milliseconds timeout) = 0;
    // Polite request: Ctrl+Break to the process group (Windows) / SIGTERM to the group (POSIX). May be ignored by the child.
    virtual void requestStop() = 0;
    // Immediate, whole tree, idempotent.
    virtual void kill() = 0;
};

using ProcessFactory = std::function<std::unique_ptr<IRobotProcess>()>;

// The real thing for this OS (process_win.cpp / process_posix.cpp).
std::unique_ptr<IRobotProcess> createSystemProcess();
ProcessFactory systemProcessFactory();

// Best-effort kill of an arbitrary process and its descendants by PID (e.g. the robot PID from Hello after the
// supervising emulator crashed and restarted). Never kills the calling process or its ancestors.
sdk::Status killProcessTree(std::int64_t pid);

// ---- helpers shared by the implementations (pure, unit-tested) --------------------------------------------------

// Splits a byte stream into lines; feed() may be called with arbitrary chunking.
class LineSplitter {
public:
    explicit LineSplitter(std::size_t maxLineBytes = 64 * 1024) : maxLine_(maxLineBytes) {}
    void feed(std::string_view chunk, const std::function<void(std::string_view)>& onLine);
    void flush(const std::function<void(std::string_view)>& onLine);  // delivers the unterminated tail

private:
    std::string buffer_;
    std::size_t maxLine_;
    bool justCut_ = false;
};

// Environment of the child: `base` (name=value) + spec.env overrides - spec.unsetEnv. Windows names are matched
// case-insensitively. Result sorted by name (Windows requires a sorted block).
std::vector<std::string> mergeEnvironment(const std::vector<std::string>& base, const ProcessSpec& spec, bool caseInsensitiveNames);

// CommandLineToArgvW-compatible quoting of one argument.
std::string quoteWindowsArgument(std::string_view arg);
// Arguments that cannot be passed to a .bat/.cmd at all: `"` (would end the quoting), `%` (expanded by cmd even inside
// quotes), CR/LF and NUL. cmd.exe metacharacters (& ^ | < > ( ) !) are safe because quoteBatchArgument() wraps them in
// quotes and the process layer runs cmd with delayed expansion forced off (/v:off).
bool isSafeForBatchFile(std::string_view arg);
// Quoting of one argument inside `cmd /d /v:off /s /c "..."`: wraps in double quotes whenever cmd would otherwise act on a
// character (space, & ^ | < > ( ) ! , ; =). Only valid for arguments accepted by isSafeForBatchFile().
std::string quoteBatchArgument(std::string_view arg);
// "lib/foo" style lookup of `program` in `pathVar` (separator ';' on Windows, ':' elsewhere, PATHEXT on Windows).
std::optional<std::filesystem::path> findOnPath(std::string_view program, std::string_view pathVar, bool windows,
                                                std::string_view pathExt = ".COM;.EXE;.BAT;.CMD");

}  // namespace wpiemu::core::launcher
