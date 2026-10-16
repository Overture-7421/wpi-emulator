// Windows supervision: the child is created suspended, put into a kill-on-close job object (so the whole tree dies with
// it) and resumed; stdout/stderr go through anonymous pipes with exactly three inheritable handles.
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <set>
#include <thread>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <tlhelp32.h>

#include "wpiemu/core/launcher/process.h"

namespace wpiemu::core::launcher {

using namespace wpiemu::sdk;
namespace fs = std::filesystem;

namespace {

std::wstring toWide(std::string_view s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
    std::wstring out(static_cast<std::size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), out.data(), n);
    return out;
}

std::string toUtf8(const wchar_t* s, std::size_t length) {
    if (length == 0) return {};
    const int n = WideCharToMultiByte(CP_UTF8, 0, s, static_cast<int>(length), nullptr, 0, nullptr, nullptr);
    std::string out(static_cast<std::size_t>(n), '\0');
    WideCharToMultiByte(CP_UTF8, 0, s, static_cast<int>(length), out.data(), n, nullptr, nullptr);
    return out;
}

std::string lastErrorText(DWORD code) {
    wchar_t* buffer = nullptr;
    const DWORD n = FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, code,
                                   0, reinterpret_cast<LPWSTR>(&buffer), 0, nullptr);
    std::string text = n != 0 ? toUtf8(buffer, n) : "error " + std::to_string(code);
    if (buffer != nullptr) LocalFree(buffer);
    while (!text.empty() && (text.back() == '\n' || text.back() == '\r' || text.back() == ' ')) text.pop_back();
    return text + " (" + std::to_string(code) + ")";
}

std::vector<std::string> currentEnvironment() {
    std::vector<std::string> out;
    wchar_t* block = GetEnvironmentStringsW();
    if (block == nullptr) return out;
    for (const wchar_t* p = block; *p != L'\0';) {
        const std::size_t len = wcslen(p);
        out.push_back(toUtf8(p, len));
        p += len + 1;
    }
    FreeEnvironmentStringsW(block);
    return out;
}

std::optional<std::string> lookupEnv(const std::vector<std::string>& env, std::string_view name) {
    for (const std::string& kv : env) {
        const std::size_t eq = kv.find('=', 1);
        if (eq == std::string::npos || eq != name.size()) continue;
        if (_strnicmp(kv.c_str(), name.data(), name.size()) == 0) return kv.substr(eq + 1);
    }
    return std::nullopt;
}

bool endsWithNoCase(const std::string& s, const char* suffix) {
    const std::size_t n = std::char_traits<char>::length(suffix);
    return s.size() >= n && _strnicmp(s.c_str() + s.size() - n, suffix, n) == 0;
}

struct Handle {
    HANDLE h = nullptr;
    Handle() = default;
    explicit Handle(HANDLE x) : h(x) {}
    Handle(const Handle&) = delete;
    Handle& operator=(const Handle&) = delete;
    ~Handle() { reset(); }
    void reset(HANDLE x = nullptr) {
        if (h != nullptr && h != INVALID_HANDLE_VALUE) CloseHandle(h);
        h = x;
    }
    HANDLE release() {
        HANDLE x = h;
        h = nullptr;
        return x;
    }
    bool valid() const { return h != nullptr && h != INVALID_HANDLE_VALUE; }
};

class WinProcess final : public IRobotProcess {
public:
    ~WinProcess() override {
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
        if (running()) GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT, static_cast<DWORD>(pid_));  // only reaches children sharing our console
    }
    void kill() override {
        killedByUs_ = true;
        std::lock_guard lock(killMutex_);
        if (job_.valid()) TerminateJobObject(job_.h, 1);
        else if (pid_ != 0 && running()) (void)killProcessTree(pid_);
    }

private:
    void readLoop(HANDLE pipe, OutputStream stream) {
        LineSplitter splitter;
        char buffer[8192];
        const auto emit = [&](std::string_view line) { sink_->line(stream, line); };
        for (;;) {
            DWORD got = 0;
            if (!ReadFile(pipe, buffer, sizeof(buffer), &got, nullptr) || got == 0) break;
            splitter.feed(std::string_view(buffer, got), emit);
        }
        splitter.flush(emit);
        ++readersDone_;
    }

    void monitorLoop() {
        WaitForSingleObject(process_.h, INFINITE);
        DWORD code = 0;
        GetExitCodeProcess(process_.h, &code);
        // grandchildren may keep the pipes open (daemons): give the readers a moment, then cancel their blocking reads
        for (int i = 0; i < 150 && readersDone_ < 2; ++i) Sleep(10);
        if (readersDone_ < 2) {
            CancelIoEx(outRead_.h, nullptr);
            CancelIoEx(errRead_.h, nullptr);
        }
        {
            std::lock_guard lock(mutex_);
            exit_.exitCode = static_cast<int>(code);
            exit_.killedByUs = killedByUs_;
            exit_.signalled = false;
            exited_ = true;
        }
        cv_.notify_all();
    }

    void joinAll() {
        if (outReader_.joinable()) outReader_.join();
        if (errReader_.joinable()) errReader_.join();
        if (monitor_.joinable()) monitor_.join();
    }

    mutable std::mutex mutex_;
    std::mutex killMutex_;
    std::condition_variable cv_;
    bool started_ = false;
    bool exited_ = false;
    ExitInfo exit_;
    std::atomic<bool> killedByUs_{false};
    std::atomic<int> readersDone_{0};
    std::int64_t pid_ = 0;
    ILogSink* sink_ = nullptr;
    Handle process_, job_, outRead_, errRead_;
    std::thread outReader_, errReader_, monitor_;
};

Status WinProcess::start(const ProcessSpec& spec, ILogSink& sink) {
    if (started_) return makeError(ErrorCode::InvalidState, "process already started");
    sink_ = &sink;

    // ---- environment and program resolution ----
    const std::vector<std::string> env = mergeEnvironment(currentEnvironment(), spec, true);
    const std::string pathVar = lookupEnv(env, "PATH").value_or("");
    const std::string pathExt = lookupEnv(env, "PATHEXT").value_or(".COM;.EXE;.BAT;.CMD");
    std::optional<fs::path> exe;
    if (!spec.workingDir.empty() && (spec.program.find('\\') != std::string::npos || spec.program.find('/') != std::string::npos) &&
        fs::path(spec.program).is_relative())
        exe = findOnPath((spec.workingDir / fs::path(spec.program)).string(), pathVar, true, pathExt);
    if (!exe) exe = findOnPath(spec.program, pathVar, true, pathExt);
    if (!exe) return makeError(ErrorCode::NotFound, "program not found: " + spec.program);
    std::error_code ec;
    const fs::path exePath = fs::absolute(*exe, ec);

    // ---- command line ----
    const std::string exeName = exePath.string();
    const bool batch = endsWithNoCase(exeName, ".bat") || endsWithNoCase(exeName, ".cmd");
    std::wstring application;
    std::string commandLine;
    if (batch) {
        for (const std::string& a : spec.args)
            if (!isSafeForBatchFile(a)) return makeError(ErrorCode::InvalidArgument, "argument not allowed for a batch file: " + a);
        if (!isSafeForBatchFile(exeName)) return makeError(ErrorCode::InvalidArgument, "unsafe batch file path: " + exeName);
        const std::string comspec = lookupEnv(env, "ComSpec").value_or("C:\\Windows\\System32\\cmd.exe");
        application = toWide(comspec);
        std::string inner = quoteBatchArgument(exeName);
        for (const std::string& a : spec.args) inner += " " + quoteBatchArgument(a);
        // /v:off: delayed expansion may be enabled by default in the registry, which would make `!` special inside quotes.
        commandLine = quoteWindowsArgument(comspec) + " /d /v:off /s /c \"" + inner + "\"";
    } else {
        application = exePath.wstring();
        commandLine = quoteWindowsArgument(exeName);
        for (const std::string& a : spec.args) commandLine += " " + quoteWindowsArgument(a);
    }
    std::wstring wideCommand = toWide(commandLine);
    wideCommand.push_back(L'\0');

    std::wstring envBlock;
    for (const std::string& kv : env) {
        envBlock += toWide(kv);
        envBlock.push_back(L'\0');
    }
    envBlock.push_back(L'\0');

    // ---- pipes (child ends inheritable) ----
    SECURITY_ATTRIBUTES sa{sizeof(sa), nullptr, TRUE};
    Handle outRd, outWr, errRd, errWr, nul;
    if (!CreatePipe(&outRd.h, &outWr.h, &sa, 0) || !CreatePipe(&errRd.h, &errWr.h, &sa, 0))
        return makeError(ErrorCode::IoError, "cannot create pipes: " + lastErrorText(GetLastError()));
    SetHandleInformation(outRd.h, HANDLE_FLAG_INHERIT, 0);
    SetHandleInformation(errRd.h, HANDLE_FLAG_INHERIT, 0);
    nul.reset(CreateFileW(L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, &sa, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr));
    if (!nul.valid()) return makeError(ErrorCode::IoError, "cannot open NUL: " + lastErrorText(GetLastError()));

    // ---- job object ----
    Handle job(CreateJobObjectW(nullptr, nullptr));
    if (!job.valid()) return makeError(ErrorCode::IoError, "cannot create job object: " + lastErrorText(GetLastError()));
    JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};
    limits.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
    if (!SetInformationJobObject(job.h, JobObjectExtendedLimitInformation, &limits, sizeof(limits)))
        return makeError(ErrorCode::IoError, "cannot configure job object: " + lastErrorText(GetLastError()));

    // ---- exactly these three handles are inherited ----
    HANDLE inherited[3] = {nul.h, outWr.h, errWr.h};
    SIZE_T attrSize = 0;
    InitializeProcThreadAttributeList(nullptr, 1, 0, &attrSize);
    std::vector<char> attrStorage(attrSize);
    auto* attrList = reinterpret_cast<LPPROC_THREAD_ATTRIBUTE_LIST>(attrStorage.data());
    if (!InitializeProcThreadAttributeList(attrList, 1, 0, &attrSize) ||
        !UpdateProcThreadAttribute(attrList, 0, PROC_THREAD_ATTRIBUTE_HANDLE_LIST, inherited, sizeof(inherited), nullptr, nullptr))
        return makeError(ErrorCode::IoError, "cannot prepare handle list: " + lastErrorText(GetLastError()));

    STARTUPINFOEXW si{};
    si.StartupInfo.cb = sizeof(si);
    si.StartupInfo.dwFlags = STARTF_USESTDHANDLES;
    si.StartupInfo.hStdInput = nul.h;
    si.StartupInfo.hStdOutput = outWr.h;
    si.StartupInfo.hStdError = errWr.h;
    si.lpAttributeList = attrList;

    PROCESS_INFORMATION pi{};
    const std::wstring cwd = spec.workingDir.empty() ? std::wstring() : spec.workingDir.wstring();
    const DWORD flags = CREATE_SUSPENDED | CREATE_UNICODE_ENVIRONMENT | EXTENDED_STARTUPINFO_PRESENT | CREATE_NEW_PROCESS_GROUP | CREATE_NO_WINDOW;
    const BOOL created = CreateProcessW(application.c_str(), wideCommand.data(), nullptr, nullptr, TRUE, flags, envBlock.data(),
                                        cwd.empty() ? nullptr : cwd.c_str(), &si.StartupInfo, &pi);
    const DWORD createError = GetLastError();
    DeleteProcThreadAttributeList(attrList);
    if (!created) return makeError(ErrorCode::IoError, "cannot start " + exeName + ": " + lastErrorText(createError));

    Handle thread(pi.hThread);
    process_.reset(pi.hProcess);
    pid_ = pi.dwProcessId;
    if (AssignProcessToJobObject(job.h, process_.h)) {
        job_.reset(job.release());
    }  // else: tree kill falls back to the PID walk in killProcessTree()
    ResumeThread(thread.h);

    outWr.reset();
    errWr.reset();
    nul.reset();
    outRead_.reset(outRd.release());
    errRead_.reset(errRd.release());
    {
        std::lock_guard lock(mutex_);
        started_ = true;
    }
    outReader_ = std::thread([this] { readLoop(outRead_.h, OutputStream::Stdout); });
    errReader_ = std::thread([this] { readLoop(errRead_.h, OutputStream::Stderr); });
    monitor_ = std::thread([this] { monitorLoop(); });
    return okStatus();
}

}  // namespace

std::unique_ptr<IRobotProcess> createSystemProcess() { return std::make_unique<WinProcess>(); }

Status killProcessTree(std::int64_t pid) {
    if (pid <= 0) return makeError(ErrorCode::InvalidArgument, "invalid pid");
    Handle snapshot(CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0));
    if (!snapshot.valid()) return makeError(ErrorCode::IoError, "cannot snapshot processes: " + lastErrorText(GetLastError()));
    std::vector<std::pair<DWORD, DWORD>> procs;  // pid, parent
    PROCESSENTRY32W entry{};
    entry.dwSize = sizeof(entry);
    for (BOOL ok = Process32FirstW(snapshot.h, &entry); ok; ok = Process32NextW(snapshot.h, &entry))
        procs.emplace_back(entry.th32ProcessID, entry.th32ParentProcessID);

    // never kill ourselves or an ancestor
    std::set<DWORD> protectedPids;
    for (DWORD cur = GetCurrentProcessId(); cur != 0 && protectedPids.insert(cur).second;) {
        DWORD parent = 0;
        for (const auto& [p, pp] : procs)
            if (p == cur) parent = pp;
        cur = parent;
    }
    const DWORD root = static_cast<DWORD>(pid);
    if (protectedPids.count(root) != 0) return makeError(ErrorCode::InvalidArgument, "refusing to kill the emulator itself");
    if (std::none_of(procs.begin(), procs.end(), [&](const auto& pr) { return pr.first == root; }))
        return makeError(ErrorCode::NotFound, "no such process: " + std::to_string(pid));

    std::vector<DWORD> order{root};  // breadth first; killed in reverse (leaves first)
    for (std::size_t i = 0; i < order.size(); ++i)
        for (const auto& [p, pp] : procs)
            if (pp == order[i] && std::find(order.begin(), order.end(), p) == order.end() && protectedPids.count(p) == 0) order.push_back(p);
    bool any = false;
    for (auto it = order.rbegin(); it != order.rend(); ++it) {
        Handle h(OpenProcess(PROCESS_TERMINATE, FALSE, *it));
        if (h.valid() && TerminateProcess(h.h, 1)) any = true;
    }
    return any ? okStatus() : Status(makeError(ErrorCode::IoError, "could not terminate process " + std::to_string(pid)));
}

}  // namespace wpiemu::core::launcher
