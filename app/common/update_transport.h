#pragma once

// The updater's only door to the network. `api.github.com` is deliberately not used (60 requests/hour per IP when
// unauthenticated): the release is found through the `releases/latest` redirect and everything else is a plain download.

#include <QString>
#include <QUrl>

#include <functional>
#include <memory>

namespace wpiemu::app {

struct HttpResult {
    bool ok = false;        // a usable answer: 2xx, or (head) any answer
    int status = 0;         // HTTP status, 0 when the request never got one
    QString error;          // human readable reason when !ok
    QString location;       // absolute Location of a redirect (head only)
    QByteArray body;        // getText only
};

// Callbacks run on the thread that owns the transport (the UI thread) and never after abort().
class IUpdateTransport {
public:
    using Done = std::function<void(const HttpResult&)>;
    using Progress = std::function<void(qint64 received, qint64 total)>;   // total is -1 when unknown

    virtual ~IUpdateTransport() = default;

    // Reports status and Location of the first answer without following a redirect.
    virtual void head(const QUrl& url, Done done) = 0;
    // A small body (at most 64 KiB); redirects to allowed hosts are followed.
    virtual void getText(const QUrl& url, Done done) = 0;
    // Streams the body to `file` (written as file + ".part", renamed when complete); redirects to allowed hosts are followed.
    virtual void download(const QUrl& url, const QString& file, Progress progress, Done done) = 0;
    // Cancels the running request (and removes a partial file). Only one request runs at a time.
    virtual void abort() = 0;
};

// https on github.com or *.githubusercontent.com (where release files are served from) and nowhere else.
bool isAllowedUpdateUrl(const QUrl& url);

std::unique_ptr<IUpdateTransport> createQtUpdateTransport(const QString& appVersion);

}  // namespace wpiemu::app
