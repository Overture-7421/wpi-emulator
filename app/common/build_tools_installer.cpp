#include "common/build_tools_installer.h"

#include <QDesktopServices>
#include <QPointer>
#include <QProcess>
#include <QStandardPaths>

namespace wpiemu::app {

namespace {

class QtBuildToolsEffects final : public IBuildToolsEffects {
public:
    bool wingetAvailable() override { return !QStandardPaths::findExecutable(QStringLiteral("winget")).isEmpty(); }

    void runWinget(const QStringList& args, std::function<void(int, const QString&)> done) override {
        cancel();
        process_ = std::make_unique<QProcess>();
        process_->setProcessChannelMode(QProcess::MergedChannels);
        QProcess* raw = process_.get();
        QObject::connect(raw, &QProcess::errorOccurred, &context_, [this, raw, done](QProcess::ProcessError e) {
            if (e == QProcess::FailedToStart) finish(raw, -1, QObject::tr("winget could not be started."), done);
        });
        QObject::connect(raw, &QProcess::finished, &context_, [this, raw, done](int code, QProcess::ExitStatus status) {
            finish(raw, status == QProcess::NormalExit ? code : -1, QString::fromLocal8Bit(raw->readAll()), done);
        });
        raw->start(QStringLiteral("winget"), args);
    }

    void cancel() override {
        if (!process_) return;
        process_->disconnect();
        process_->kill();
        process_->waitForFinished(2000);
        process_.reset();
    }

    bool openUrl(const QUrl& url) override { return QDesktopServices::openUrl(url); }

private:
    void finish(QProcess* raw, int code, const QString& output, const std::function<void(int, const QString&)>& done) {
        if (process_.get() != raw) return;
        process_.release()->deleteLater();
        done(code, output);
    }

    QObject context_;
    std::unique_ptr<QProcess> process_;
};

}  // namespace

std::unique_ptr<IBuildToolsEffects> createQtBuildToolsEffects() { return std::make_unique<QtBuildToolsEffects>(); }

BuildToolsInstaller::BuildToolsInstaller(const ISystemProbe& probe, std::unique_ptr<IBuildToolsEffects> effects, QObject* parent)
    : QObject(parent), probe_(probe), effects_(std::move(effects)) {}

BuildToolsInstaller::~BuildToolsInstaller() {
    if (busy()) effects_->cancel();
}

QStringList BuildToolsInstaller::wingetArguments() {
    return {QStringLiteral("install"),
            QStringLiteral("-e"),
            QStringLiteral("--id"),
            QStringLiteral("Microsoft.VisualStudio.2022.BuildTools"),
            QStringLiteral("--accept-source-agreements"),
            QStringLiteral("--accept-package-agreements"),
            QStringLiteral("--override"),
            QStringLiteral("--passive --wait --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended")};
}

QUrl BuildToolsInstaller::downloadPage() { return QUrl(QStringLiteral("https://aka.ms/vs/17/release/vs_BuildTools.exe")); }

void BuildToolsInstaller::setState(BuildToolsState state, QString message) {
    state_ = state;
    message_ = std::move(message);
    emit stateChanged();
}

void BuildToolsInstaller::start() {
    if (busy() || !available()) return;
    const quint64 gen = ++generation_;
    if (!effects_->wingetAvailable()) {
        const bool opened = effects_->openUrl(downloadPage());
        setState(opened ? BuildToolsState::OpenedDownload : BuildToolsState::Failed,
                 opened ? tr("The Visual Studio Build Tools installer was downloaded by your browser. Run it, tick \"Desktop development with C++\", "
                             "then press Re-check.")
                        : tr("Could not open %1. Download the Visual Studio 2022 Build Tools yourself and tick \"Desktop development with C++\".")
                              .arg(downloadPage().toString()));
        emit finished(false);
        return;
    }
    setState(BuildToolsState::Installing, tr("Installing the C++ build tools. Follow the installer window and allow the administrator prompt."));
    effects_->runWinget(wingetArguments(), [this, gen](int code, const QString& output) {
        if (gen != generation_) return;
        if (code == 0) {
            setState(BuildToolsState::Done, tr("The C++ build tools are installed."));
            emit finished(true);
        } else {
            // A non-zero code can still mean the tools are there (for example a restart is pending): Re-check tells the truth.
            setState(BuildToolsState::Failed, tr("The installer ended with code %1. Press Re-check: it may have installed anyway.\n%2").arg(code).arg(output.trimmed().right(300)));
            emit finished(false);
        }
    });
}

void BuildToolsInstaller::cancel() {
    if (!busy()) return;
    ++generation_;
    effects_->cancel();
    setState(BuildToolsState::Idle, {});
    emit finished(false);
}

}  // namespace wpiemu::app
