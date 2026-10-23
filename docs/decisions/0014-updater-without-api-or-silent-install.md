# The updater needs no API and installs nothing silently

Read this when changing how the app finds or runs an update.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Rationale
Unauthenticated API limits are shared by classrooms and team networks. Replacing an installed
application without a user's explicit action would also risk interrupting work.

## Decision
Discover public releases without an API token, verify downloads against published SHA-256
sidecars, and require an explicit user action before platform-specific update handling.

## Trade-offs
A checksum proves that a download matches its sidecar, not who produced it. The release process
must preserve the names, sidecars and deployment assets the updater expects; code signing would
provide a stronger publisher identity.

Current update behavior and release conventions are in [Application](../reference/app.md) and
[Releasing](../releasing.md#in-app-updates).
