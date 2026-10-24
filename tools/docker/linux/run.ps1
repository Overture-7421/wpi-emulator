# Builds / tests / packages the Linux target in a ubuntu:24.04 container (see docs/BUILDING.md).
#   .\tools\docker\linux\run.ps1 [all|configure|build|test|install|appimage|shell] [-Rebuild] [-CtestArgs '-R "^launcher"']
# Build output and downloads live in the named volume wpiemu-linux-build, so repeated runs are incremental.
# Copy artifacts out with: docker run --rm -v wpiemu-linux-build:/build -v ${PWD}:/out --entrypoint cp wpiemu-linux-ci -r /build/release/pkg /out/
param(
    [string]$Step = "all",
    [switch]$Rebuild,
    [string]$CtestArgs = ""
)
$ErrorActionPreference = "Stop"
$root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$image = "wpiemu-linux-ci"

if ($Rebuild -or -not (docker image ls -q $image)) {
    docker build -t $image $PSScriptRoot
    if ($LASTEXITCODE -ne 0) { throw "docker build failed" }
}

$tty = if ($Step -eq "shell") { @("-it") } else { @() }
docker run --rm @tty `
    -v "${root}:/host-src:ro" `
    -v wpiemu-linux-build:/build `
    -e "CTEST_ARGS=$CtestArgs" `
    $image $Step
exit $LASTEXITCODE
