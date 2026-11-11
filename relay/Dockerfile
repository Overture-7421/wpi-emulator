# syntax=docker/dockerfile:1
# The practice-match relay (wpiemu_relay) as a small Linux image. The build context is the repository root:
#
#   docker build -f relay/Dockerfile -t wpiemu-relay:dev .
#   docker run --rm -p 7777:7777/udp wpiemu-relay:dev
#
# Stage 1 compiles a relay-only tree (WPIEMU_RELAY_ONLY: sdk, ipc, net and relay; no Qt, Box3D, WPILib or SDL), stage 2 holds
# the stripped binaries. Both stages use the same Ubuntu release because the binary links glibc dynamically (libstdc++ and
# libgcc are linked in, see relay/CMakeLists.txt). A distroless/Debian runtime would have an older glibc than this build.
# The base image tags are floating; the release pipeline resolves and records the image digest and Bicep pulls by digest.
# Configuration is by environment variable (WPIEMU_RELAY_*, see relay/include/wpiemu/relay/relay_options.h).

FROM ubuntu:24.04 AS build

ENV DEBIAN_FRONTEND=noninteractive
# Package versions are not pinned: the Ubuntu archive drops superseded versions, so a pin would break rebuilds. Reproducibility
# comes from the pipeline pinning the base image by digest.
# hadolint ignore=DL3008
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates cmake g++-14 gcc-14 git ninja-build \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY . /src

# Release build with the usual Linux hardening. Warnings stay errors; tests and the desktop targets are not part of the image.
# FetchContent clones ENet6 and Monocypher at their pinned tags over https.
ARG HARDENING_FLAGS="-fstack-protector-strong -fstack-clash-protection -D_FORTIFY_SOURCE=3"
ARG LINKER_FLAGS="-Wl,-z,relro -Wl,-z,now -Wl,-z,noexecstack"
RUN cmake -S . -B /build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_C_COMPILER=gcc-14 -DCMAKE_CXX_COMPILER=g++-14 \
        "-DCMAKE_C_FLAGS=${HARDENING_FLAGS}" "-DCMAKE_CXX_FLAGS=${HARDENING_FLAGS}" \
        "-DCMAKE_EXE_LINKER_FLAGS=${LINKER_FLAGS}" \
        -DWPIEMU_RELAY_ONLY=ON -DWPIEMU_BUILD_TESTS=OFF -DWPIEMU_BUILD_APP=OFF -DWPIEMU_WARNINGS_AS_ERRORS=ON \
    && cmake --build /build --target wpiemu_relay wpiemu_relay_probe \
    && cmake --install /build --component Relay --strip --prefix /stage

FROM ubuntu:24.04 AS runtime

ARG VERSION=0.0.0-dev
ARG REVISION=unknown
ARG SOURCE_URL=https://github.com/Overture-7421/wpi-emulator
LABEL org.opencontainers.image.title="wpiemu-relay" \
      org.opencontainers.image.description="WPI Emulator practice-match relay (ENet over UDP)" \
      org.opencontainers.image.source="${SOURCE_URL}" \
      org.opencontainers.image.revision="${REVISION}" \
      org.opencontainers.image.version="${VERSION}" \
      org.opencontainers.image.base.name="ubuntu:24.04"

# The floating base tag can lag behind Ubuntu security updates (Trivy fails the pipeline on fixable HIGH findings), so install them.
# hadolint ignore=DL3005
RUN apt-get update \
    && apt-get upgrade -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

# A fixed numeric uid/gid lets orchestrators verify "runs as non-root" without resolving names.
RUN groupadd --system --gid 10001 relay \
    && useradd --system --uid 10001 --gid 10001 --no-create-home --home-dir /nonexistent --shell /usr/sbin/nologin relay

COPY --from=build /stage/bin/wpiemu_relay /stage/bin/wpiemu_relay_probe /usr/local/bin/

ENV WPIEMU_RELAY_PORT=7777 \
    WPIEMU_RELAY_LOG=json
USER 10001:10001
EXPOSE 7777/udp

# Docker only: Azure Container Instances ignores HEALTHCHECK and uses the liveness probe in infra/relay/modules/relay.bicep,
# which runs the same command. --selfcheck reads WPIEMU_RELAY_PORT, asks the running relay about a random room and expects
# a refusal, so it creates no room. One check per 30 s stays well below the relay's failed-join lockout.
HEALTHCHECK --interval=30s --timeout=5s --start-period=10s --retries=3 \
    CMD ["/usr/local/bin/wpiemu_relay", "--selfcheck"]

# Binding a port below 1024 (for example 443) as uid 10001 fails ("cannot bind") unless the container's network namespace has
# net.ipv4.ip_unprivileged_port_start=0: Docker sets that by default, `docker run --sysctl net.ipv4.ip_unprivileged_port_start=0`
# does it explicitly. A runtime that cannot set sysctls (Azure Container Instances) needs the file capability instead, in the
# runtime stage: `setcap cap_net_bind_service=+ep /usr/local/bin/wpiemu_relay` (package libcap2-bin; it works as non-root unless
# no-new-privs is set). Both were verified on Linux; the default port 7777 needs neither.
ENTRYPOINT ["/usr/local/bin/wpiemu_relay"]
