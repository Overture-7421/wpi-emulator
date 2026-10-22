#include "common/update_transport.h"

#include <QFile>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QObject>
#include <QPointer>

namespace wpiemu::app {
namespace {

constexpr qint64 kMaxTextBytes = 64 * 1024;
constexpr qint64 kMaxDownloadBytes = 600ll * 1024 * 1024;
constexpr int kMaxRedirects = 5;
constexpr int kTimeoutMs = 30000;   // without any data

bool isRedirect(int status) { return status == 301 || status == 302 || status == 303 || status == 307 || status == 308; }

class QtUpdateTransport final : public QObject, public IUpdateTransport {
public:
    explicit QtUpdateTransport(QString appVersion) : userAgent_(QStringLiteral("wpi-emulator/") + appVersion) {
        nam_.setRedirectPolicy(QNetworkRequest::ManualRedirectPolicy);
    }

    void head(const QUrl& url, Done done) override { start(Kind::Head, url, {}, nullptr, std::move(done)); }
    void getText(const QUrl& url, Done done) override { start(Kind::Text, url, {}, nullptr, std::move(done)); }
    void download(const QUrl& url, const QString& file, Progress progress, Done done) override {
        start(Kind::File, url, file, std::move(progress), std::move(done));
    }
    void abort() override { cancel(); }

private:
    enum class Kind { Head, Text, File };
    struct Job {
        Kind kind = Kind::Head;
        QUrl url;
        QString file;
        Progress progress;
        Done done;
        int redirects = 0;
        QByteArray body;
        std::unique_ptr<QFile> out;
        qint64 written = 0;
        bool failed = false;
        QString failure;
    };

    void start(Kind kind, const QUrl& url, const QString& file, Progress progress, Done done) {
        cancel();
        job_ = std::make_unique<Job>();
        job_->kind = kind;
        job_->url = url;
        job_->file = file;
        job_->progress = std::move(progress);
        job_->done = std::move(done);
        if (!isAllowedUpdateUrl(url)) {
            finish(false, 0, tr("The address %1 is not an allowed update source").arg(url.toString()), {});
            return;
        }
        issue();
    }

    void issue() {
        QNetworkRequest request(job_->url);
        request.setHeader(QNetworkRequest::UserAgentHeader, userAgent_);
        request.setTransferTimeout(kTimeoutMs);
        request.setAttribute(QNetworkRequest::CookieLoadControlAttribute, QNetworkRequest::Manual);
        request.setAttribute(QNetworkRequest::CookieSaveControlAttribute, QNetworkRequest::Manual);
        request.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::ManualRedirectPolicy);
        QNetworkReply* reply = job_->kind == Kind::Head ? nam_.head(request) : nam_.get(request);
        reply_ = reply;
        const Job* job = job_.get();

        connect(reply, &QNetworkReply::readyRead, this, [this, reply, job] {
            if (reply_ != reply || job_.get() != job) return;
            onData(reply);
        });
        connect(reply, &QNetworkReply::downloadProgress, this, [this, reply, job](qint64 received, qint64 total) {
            if (reply_ != reply || job_.get() != job || job_->kind != Kind::File || statusOf(reply) != 200) return;
            if (job_->progress) job_->progress(received, total);
        });
        connect(reply, &QNetworkReply::finished, this, [this, reply, job] {
            if (reply_ != reply || job_.get() != job) {
                reply->deleteLater();
                return;
            }
            onFinished(reply);
        });
    }

    static int statusOf(QNetworkReply* reply) { return reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt(); }

    void onData(QNetworkReply* reply) {
        if (job_->failed || job_->kind == Kind::Head) return;
        const int status = statusOf(reply);
        if (status != 200) return;   // the body of an error or a redirect is not wanted
        const QByteArray chunk = reply->readAll();
        if (job_->kind == Kind::Text) {
            job_->body += chunk;
            if (job_->body.size() > kMaxTextBytes) fail(tr("The answer is larger than expected"), reply);
            return;
        }
        job_->written += chunk.size();
        if (job_->written > kMaxDownloadBytes) return fail(tr("The download is larger than expected"), reply);
        if (!job_->out) {
            job_->out = std::make_unique<QFile>(job_->file + QStringLiteral(".part"));
            if (!job_->out->open(QIODevice::WriteOnly | QIODevice::Truncate))
                return fail(tr("Cannot write %1: %2").arg(job_->file, job_->out->errorString()), reply);
        }
        if (job_->out->write(chunk) != chunk.size()) fail(tr("Cannot write %1: %2").arg(job_->file, job_->out->errorString()), reply);
    }

    void fail(const QString& why, QNetworkReply* reply) {
        job_->failed = true;
        job_->failure = why;
        reply->abort();
    }

    void onFinished(QNetworkReply* reply) {
        const int status = statusOf(reply);
        if (job_->failed) return finish(false, status, job_->failure, {}, reply);

        if (job_->kind == Kind::Head) {
            QString location;
            if (isRedirect(status)) location = reply->url().resolved(QUrl(QString::fromUtf8(reply->rawHeader("Location")))).toString();
            if (reply->error() != QNetworkReply::NoError && status == 0) return finish(false, 0, reply->errorString(), {}, reply);
            return finish(true, status, {}, location, reply);
        }
        if (isRedirect(status)) {
            const QUrl next = reply->url().resolved(QUrl(QString::fromUtf8(reply->rawHeader("Location"))));
            if (++job_->redirects > kMaxRedirects) return finish(false, status, tr("Too many redirects"), {}, reply);
            if (!isAllowedUpdateUrl(next)) return finish(false, status, tr("Redirected to %1, which is not an allowed update source").arg(next.toString()), {}, reply);
            job_->url = next;
            reply_.clear();
            reply->deleteLater();
            return issue();
        }
        if (reply->error() != QNetworkReply::NoError && status == 0) return finish(false, 0, reply->errorString(), {}, reply);
        if (status < 200 || status >= 300) return finish(false, status, tr("GitHub answered HTTP %1").arg(status), {}, reply);
        finish(true, status, {}, {}, reply);
    }

    void cancel() {
        if (!job_) return;
        QPointer<QNetworkReply> reply = reply_;
        auto partial = std::move(job_->out);
        const QString file = job_->file;
        job_.reset();
        reply_.clear();
        if (reply) {
            reply->abort();
            reply->deleteLater();
        }
        if (partial) {
            partial->close();
            QFile::remove(file + QStringLiteral(".part"));
        }
    }

    void finish(bool ok, int status, const QString& error, const QString& location, QNetworkReply* reply = nullptr) {
        std::unique_ptr<Job> job = std::move(job_);
        reply_.clear();
        if (reply) reply->deleteLater();
        HttpResult result;
        result.status = status;
        result.location = location;
        result.ok = ok;
        result.error = error;
        if (ok && job->kind == Kind::Text) result.body = job->body;
        if (job->kind == Kind::File) {
            if (job->out) job->out->close();
            const QString part = job->file + QStringLiteral(".part");
            if (ok) {
                QFile::remove(job->file);
                if (!QFile::rename(part, job->file)) {
                    result.ok = false;
                    result.error = tr("Cannot save %1").arg(job->file);
                }
            }
            if (!result.ok) QFile::remove(part);
        }
        if (job->done) job->done(result);
    }

    QNetworkAccessManager nam_;
    QString userAgent_;
    std::unique_ptr<Job> job_;
    QPointer<QNetworkReply> reply_;
};

}  // namespace

bool isAllowedUpdateUrl(const QUrl& url) {
    if (!url.isValid() || url.scheme() != QLatin1String("https")) return false;
    if (!url.userInfo().isEmpty() || (url.port() != -1 && url.port() != 443)) return false;
    const QString host = url.host().toLower();
    return host == QLatin1String("github.com") || host.endsWith(QLatin1String(".githubusercontent.com"));
}

std::unique_ptr<IUpdateTransport> createQtUpdateTransport(const QString& appVersion) {
    return std::make_unique<QtUpdateTransport>(appVersion);
}

}  // namespace wpiemu::app
