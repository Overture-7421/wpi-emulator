# The updater needs no API and installs nothing silently

Read this when changing how the app finds, downloads or runs a newer release.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Context
Users run an installed app and should hear about a fix without watching the repository. The GitHub
REST API allows 60 unauthenticated requests per hour per IP, which a classroom or team network shares
among every machine that starts the app. The installers are not code-signed, so a checksum proves the
download is intact but not who built it. An update that replaces a running app must still let the user
save their work.

## Decision
The app only ever asks `github.com`. The release is found from the redirect of the repository's
`releases/latest` page, which is never a draft or pre-release; the installer name follows from the version
and platform, and the `.sha256` sidecar beside it is the only trust anchor. Nothing is installed without
a click on the Update button in the menu bar, and a download is discarded unless its SHA-256 matches. The
updater is a service in the app layer with its network and system effects behind interfaces, so the SDK,
core and plugins never see it. What an update means follows from how the app was installed: Windows runs
the installer after the app quits, macOS opens the disk image, an AppImage is saved beside the old one,
and a portable copy only opens the release page. Development builds never check.

## Consequences
No token or rate limit is involved, and the check is one small request per start, which the user can turn
off. The release process now owns two conventions the app depends on: the installer file names and the
checksum sidecar. Signed installers would let the app verify who built an update, and would be the next
step. Qt's TLS backend must ship with every package, and Linux needs the system OpenSSL 3. The contract
is in [Releasing](../releasing.md#in-app-updates) and the
[app reference](../reference/app.md#updates).

Next: [User guide](../USER_GUIDE.md).
