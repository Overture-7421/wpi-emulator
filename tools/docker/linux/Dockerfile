# Linux build/test/package environment that matches the release runner (ubuntu-22.04, Qt 6.8.3).
# Usage: see tools/docker/linux/run.ps1 and docs/BUILDING.md.
FROM ubuntu:22.04

ARG QT_VERSION=6.8.3
ENV DEBIAN_FRONTEND=noninteractive \
    QT_ROOT_DIR=/opt/Qt/${QT_VERSION}/gcc_64 \
    QT_QPA_PLATFORM=offscreen \
    LANG=C.UTF-8

# Same package list as .github/workflows/release.yml, plus the toolchain the runner image ships.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential git curl ca-certificates file rsync python3 python3-pip ninja-build \
        libgl1-mesa-dev libxkbcommon-dev libxkbcommon-x11-0 \
        libxcb-cursor0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-render-util0 \
        libxcb-xinerama0 libxcb-shape0 libxcb-xkb1 libegl1 libvulkan-dev libfontconfig1 \
        libx11-dev libxext-dev libxrandr-dev libxi-dev libxcursor-dev libxfixes-dev libxss-dev \
        libudev-dev libasound2-dev desktop-file-utils gdb libdbus-1-3 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu 22.04 ships CMake 3.22; the presets need 3.25+.
RUN pip3 install --no-cache-dir cmake aqtinstall \
    && python3 -m aqt install-qt linux desktop ${QT_VERSION} linux_gcc_64 -O /opt/Qt -m qtquick3d qtshadertools qtmultimedia

ENV LD_LIBRARY_PATH=/opt/Qt/${QT_VERSION}/gcc_64/lib

COPY ci.sh /usr/local/bin/wpiemu-ci
RUN sed -i 's/\r$//' /usr/local/bin/wpiemu-ci && chmod +x /usr/local/bin/wpiemu-ci

WORKDIR /work
ENTRYPOINT ["/usr/local/bin/wpiemu-ci"]
CMD ["all"]
