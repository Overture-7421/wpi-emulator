# Prefer a working local Java; download only as a fallback

Read this when changing which Java runs a robot project.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Rationale
Requiring every mentor to install WPILib and a compatible JDK raises the setup barrier. Conversely,
always downloading Java would be wasteful and surprising to teams that already have a working
season toolchain.

## Decision
Search existing Java sources first, judge candidates against the project's season and Gradle
requirements, and offer a pinned managed JDK only when a known season lacks a usable local one.

## Trade-offs
The season-to-Java and download metadata need maintenance, and unsupported seasons do not get a
guessed download. A downloaded JDK also requires an initial network transfer.

Current selection and provisioning rules are in [Runtime](../reference/runtime.md) and
[Application](../reference/app.md).
