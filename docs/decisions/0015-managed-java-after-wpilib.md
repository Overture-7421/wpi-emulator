# The emulator finds a working Java first and downloads one only when none works

Read this when changing which Java runs a robot project, or how the app helps a computer without WPILib.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Context
Teammates who never write robot code cannot be asked to install WPILib, a JDK and Visual Studio. One failed twice on a Java
project: first on the Java version, then on `Expected at least 14.40, but found 14.36` although Visual Studio was current. That
message comes from WPILib's `checkMsvcRuntime`, which reads the `msvcp140.dll` the JVM loaded; Windows prefers the copy in the JDK's
own `bin` folder, so an older JDK build defeats any installed runtime. Separately, any Java 17 or newer was accepted although the
project's Gradle runs on Java 23 at most. People who do have WPILib must see no change.

## Decision
One ordered search, shared by the setup check and the launch: the Settings folder, the project season's WPILib JDK (trusted as
shipped), another installed season's, `JAVA_HOME`, `java` on `PATH`, then the emulator's own. Every candidate except the first
two is judged by the same pure rules: a real JDK, the season's Java or newer, not newer than the wrapper's Gradle supports, and on
Windows a Visual C++ runtime of at least 14.40. The emulator's Java is last, is a pinned Temurin downloaded from github.com after a
click, verified by SHA-256 and kept per user, and exists only for seasons `versions.json` maps to a Java major; a season it does
not know gets no download. The same pins say which runtime a JDK needs, and the Windows installer ships a current one that is
copied into the downloaded Java only if its own would load an old one. Gradle is started with `org.gradle.java.home` so a path in the
project's `gradle.properties` cannot override the choice. Known launch failures are classified from the output and explained.

## Consequences
With WPILib installed nothing is downloaded, offered or shown. A first run on a clean computer needs the internet twice (Java,
then Gradle's libraries). The pins need an edit when a Java security release or a new season arrives; `check_versions.py` checks
them. A JDK that fails the rules is skipped even if it would have worked, and the Settings folder is the way to insist. C++ projects
still need Microsoft's build tools: the app starts their installer on request and never installs silently. The rules are in the
[runtime](../reference/runtime.md#launcher-boundary) and [app](../reference/app.md#emulator-java-and-build-tools) references.

Next: [User guide](../USER_GUIDE.md).
