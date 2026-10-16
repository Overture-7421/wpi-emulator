// POSIX supervision: fork/exec into a NEW PROCESS GROUP (so the whole tree can be signalled), stdout/stderr through pipes,
// exec failures reported back through a close-on-exec pipe. Everything the child needs is prepared before fork() so the
// child only calls async-signal-safe functions.
#include <atomic>
#include <cerrno>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "wpiemu/core/launcher/process.h"

extern char** environ;

namespace wpiemu::core::launcher {

using namespace wpiemu::sdk;
namespace fs = std::filesystem;

namespace {

std::vector<std::string> currentEnvironment() {
    std::vector<std::string> out;
    for (char** e = environ; e != nullptr && *e != nullptr; ++e) out.emplace_back(*e);
    return out;
}

std::optional<std::string> lookupEnv(const std::vector<std::string>& env, std::string_view name) {
    for (const std::string& kv : env)
        if (kv.size() > name.size() && kv.compare(0, name.size(), name) == 0 && kv[name.size()] == '=') return kv.substr(name.size() + 1);
    return std::nullopt;
}

void setCloexec(int fd) { fcntl(fd, F_SETFD, fcntl(fd, F_GETFD) | FD_CLOEXEC); }

struct Fd {
    int fd = -1;
    Fd() = default;
    explicit Fd(int f) : fd(f) {}
    Fd(const Fd&) = delete;
    Fd& operator=(const Fd&) = delete;
    ~Fd() { reset(); }
    void reset(int f = -1) {
        if (fd >= 0) ::close(fd);
        fd = f;
    }
    int release() {
        const int f = fd;
        fd = -1;
        return f;
    }
};

class PosixProcess final : public IRobotProcess {
public:
    ~PosixProcess() override {
        kill();
        joinAll();
    }

    Status start(const ProcessSpec& spec, ILogSink& sink) override;

    bool running() const override {
        std::lock_guard lock(mutex_);
        return started_ && !exited_;
    }
    std::int64_t pid() const override { return pid_; }
    std::optional<ExitInfo> exitInfo() const override {
        std::lock_guard lock(mutex_);
        return exited_ ? std::optional<ExitInfo>(exit_) : std::nullopt;
    }
    bool waitForExit(std::chrono::milliseconds timeout) override {
        std::unique_lock lock(mutex_);
        if (!started_) return true;
        return cv_.wait_for(lock, timeout, [this] { return exited_; });
    }
    void requestStop() override {
        if (pid_ > 0 && running()) ::killpg(static_cast<pid_t>(pid_), SIGTERM);
    }
    void kill() override {
        killedByUs_ = true;
        if (pid_ > 0) ::killpg(static_cast<pid_t>(pid_), SIGKILL);  // group leader == child pid; also reaps descendants that stayed in the group
    }

private:
    void readLoop(int fd, OutputStream stream) {
        LineSplitter splitter;
        char buffer[8192];
        const auto emit = [&](std::string_view line) { sink_->line(stream, line); };
        for (;;) {
            pollfd p{fd, POLLIN, 0};
            const int r = ::poll(&p, 1, 100);
            if (r < 0 && errno == EINTR) continue;
            if (r < 0) break;
            if (r == 0) {
                if (cancelReaders_) break;  // process gone and a grandchild keeps the pipe open
                continue;
            }
            const ssize_t n = ::read(fd, buffer, sizeof(buffer));
            if (n < 0 && errno == EINTR) continue;
            if (n <= 0) break;
            splitter.feed(std::string_view(buffer, static_cast<std::size_t>(n)), emit);
        }
        splitter.flush(emit);
        ++readersDone_;
    }

    void monitorLoop() {
        int status = 0;
        pid_t r;
        do {
            r = ::waitpid(static_cast<pid_t>(pid_), &status, 0);
        } while (r < 0 && errno == EINTR);
        ExitInfo info;
        if (r < 0) info.exitCode = -1;
        else if (WIFEXITED(status)) info.exitCode = WEXITSTATUS(status);
        else if (WIFSIGNALED(status)) {
            info.exitCode = 128 + WTERMSIG(status);
            info.signalled = true;
        }
        for (int i = 0; i < 150 && readersDone_ < 2; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(10));
        cancelReaders_ = true;
        {
            std::lock_guard lock(mutex_);
            info.killedByUs = killedByUs_;
            exit_ = info;
            exited_ = true;
        }
        cv_.notify_all();
    }

    void joinAll() {
        if (outReader_.joinable()) outReader_.join();
        if (errReader_.joinable()) errReader_.join();
        if (monitor_.joinable()) monitor_.join();
        outFd_.reset();
        errFd_.reset();
    }

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool started_ = false;
    bool exited_ = false;
    ExitInfo exit_;
    std::atomic<bool> killedByUs_{false};
    std::atomic<bool> cancelReaders_{false};
    std::atomic<int> readersDone_{0};
    std::int64_t pid_ = 0;
    ILogSink* sink_ = nullptr;
    Fd outFd_, errFd_;
    std::thread outReader_, errReader_, monitor_;
};

Status PosixProcess::start(const ProcessSpec& spec, ILogSink& sink) {
    if (started_) return makeError(ErrorCode::InvalidState, "process already started");
    sink_ = &sink;

    const std::vector<std::string> env = mergeEnvironment(currentEnvironment(), spec, false);
    const std::string pathVar = lookupEnv(env, "PATH").value_or("/usr/local/bin:/usr/bin:/bin");
    std::optional<fs::path> exe;
    if (!spec.workingDir.empty() && spec.program.find('/') != std::string::npos && fs::path(spec.program).is_relative())
        exe = findOnPath((spec.workingDir / fs::path(spec.program)).string(), pathVar, false);
    if (!exe) exe = findOnPath(spec.program, pathVar, false);
    if (!exe) return makeError(ErrorCode::NotFound, "program not found (or not executable): " + spec.program);
    std::error_code ec;
    const std::string exePath = fs::absolute(*exe, ec).string();

    std::vector<std::string> argvStrings{spec.program};
    argvStrings.insert(argvStrings.end(), spec.args.begin(), spec.args.end());
    std::vector<char*> argv;
    for (std::string& a : argvStrings) argv.push_back(a.data());
    argv.push_back(nullptr);
    std::vector<std::string> envStrings = env;
    std::vector<char*> envp;
    for (std::string& e : envStrings) envp.push_back(e.data());
    envp.push_back(nullptr);
    const std::string cwd = spec.workingDir.string();

    int outPipe[2], errPipe[2], failPipe[2];
    if (::pipe(outPipe) != 0) return makeError(ErrorCode::IoError, std::string("pipe: ") + std::strerror(errno));
    Fd outR(outPipe[0]), outW(outPipe[1]);
    if (::pipe(errPipe) != 0) return makeError(ErrorCode::IoError, std::string("pipe: ") + std::strerror(errno));
    Fd errR(errPipe[0]), errW(errPipe[1]);
    if (::pipe(failPipe) != 0) return makeError(ErrorCode::IoError, std::string("pipe: ") + std::strerror(errno));
    Fd failR(failPipe[0]), failW(failPipe[1]);
    for (int fd : {outR.fd, errR.fd, failR.fd, failW.fd}) setCloexec(fd);
    const int devNull = ::open("/dev/null", O_RDONLY | O_CLOEXEC);

    const pid_t child = ::fork();
    if (child < 0) {
        if (devNull >= 0) ::close(devNull);
        return makeError(ErrorCode::IoError, std::string("fork: ") + std::strerror(errno));
    }
    if (child == 0) {
        ::setpgid(0, 0);  // new group: pid == pgid, signalled as a whole
        if (devNull >= 0) ::dup2(devNull, STDIN_FILENO);
        ::dup2(outW.fd, STDOUT_FILENO);
        ::dup2(errW.fd, STDERR_FILENO);
        if (!cwd.empty() && ::chdir(cwd.c_str()) != 0) {
            const int e = errno;
            const ssize_t ignored = ::write(failW.fd, &e, sizeof(e));
            (void)ignored;
            ::_exit(127);
        }
        ::execve(exePath.c_str(), argv.data(), envp.data());
        const int e = errno;
        const ssize_t ignored = ::write(failW.fd, &e, sizeof(e));
        (void)ignored;
        ::_exit(127);
    }
    ::setpgid(child, child);  // also from the parent: closes the race with the child's own call
    if (devNull >= 0) ::close(devNull);
    outW.reset();
    errW.reset();
    failW.reset();

    int childErrno = 0;
    ssize_t n;
    do {
        n = ::read(failR.fd, &childErrno, sizeof(childErrno));
    } while (n < 0 && errno == EINTR);
    if (n == static_cast<ssize_t>(sizeof(childErrno))) {
        int status = 0;
        ::waitpid(child, &status, 0);
        return makeError(ErrorCode::IoError, "cannot start " + exePath + ": " + std::strerror(childErrno));
    }

    pid_ = child;
    outFd_.reset(outR.release());
    errFd_.reset(errR.release());
    {
        std::lock_guard lock(mutex_);
        started_ = true;
    }
    outReader_ = std::thread([this] { readLoop(outFd_.fd, OutputStream::Stdout); });
    errReader_ = std::thread([this] { readLoop(errFd_.fd, OutputStream::Stderr); });
    monitor_ = std::thread([this] { monitorLoop(); });
    return okStatus();
}

}  // namespace

std::unique_ptr<IRobotProcess> createSystemProcess() { return std::make_unique<PosixProcess>(); }

Status killProcessTree(std::int64_t pid) {
    if (pid <= 1) return makeError(ErrorCode::InvalidArgument, "invalid pid");
    const pid_t target = static_cast<pid_t>(pid);
    const pid_t self = ::getpid();
    if (target == self || target == ::getppid()) return makeError(ErrorCode::InvalidArgument, "refusing to kill the emulator itself or its parent");
    if (::kill(target, 0) != 0 && errno == ESRCH) return makeError(ErrorCode::NotFound, "no such process: " + std::to_string(pid));

    // the group of the target, unless it is our own group (then only the process itself)
    const pid_t group = ::getpgid(target);
    bool any = false;
    if (group > 1 && group != ::getpgrp()) any = ::killpg(group, SIGKILL) == 0;
    if (::kill(target, SIGKILL) == 0) any = true;
    return any ? okStatus() : Status(makeError(ErrorCode::IoError, std::string("kill: ") + std::strerror(errno)));
}

}  // namespace wpiemu::core::launcher
