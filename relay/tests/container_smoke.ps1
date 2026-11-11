#requires -Version 7.0
<#
.SYNOPSIS
    Builds the relay image, runs it and checks that it is healthy and stops cleanly.

.DESCRIPTION
    Needs Docker. Run it from anywhere on a Linux, macOS or Windows machine (CI runs it on a Linux runner):

        pwsh relay/tests/container_smoke.ps1

    Steps: docker build -f relay/Dockerfile (context = repository root), docker run -d -p <HostPort>:7777/udp, wait until the
    relay answers `wpiemu_relay --selfcheck` inside the container (and Docker reports it healthy), run the round-trip probe
    when the image's wpiemu_relay_probe supports `--relay`, then docker stop and assert exit code 0 (a clean SIGTERM stop).
    The container and the image are removed on success and on failure.
#>
[CmdletBinding()]
param(
    [string]$Image = 'wpiemu-relay:smoke',
    [string]$Name = 'wpiemu-relay-smoke',
    [ValidateRange(1, 65535)][int]$HostPort = 17777,
    [ValidateRange(1, 600)][int]$TimeoutSeconds = 60,
    [switch]$SkipBuild,
    [switch]$KeepImage
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..' '..')).Path
$containerStarted = $false
$failure = $null

function Invoke-Docker {
    param([Parameter(Mandatory)][string[]]$Arguments, [switch]$AllowFailure)
    $output = & docker @Arguments 2>&1
    $code = $LASTEXITCODE
    if ($code -ne 0 -and -not $AllowFailure) {
        throw "docker $($Arguments -join ' ') failed with exit code ${code}:`n$($output -join "`n")"
    }
    [pscustomobject]@{ ExitCode = $code; Output = ($output -join "`n").Trim() }
}

function Write-Step([string]$Message) { Write-Host "==> $Message" }

try {
    if (-not (Get-Command docker -ErrorAction SilentlyContinue)) { throw 'docker was not found on PATH' }

    if (-not $SkipBuild) {
        Write-Step "docker build -f relay/Dockerfile -t $Image ."
        $sha = (& git -C $repoRoot rev-parse --short HEAD 2>$null)
        $revision = if ($LASTEXITCODE -eq 0 -and $sha) { $sha } else { 'unknown' }
        Invoke-Docker @('build', '-f', (Join-Path $repoRoot 'relay' 'Dockerfile'), '-t', $Image,
            '--build-arg', "REVISION=$revision", $repoRoot) | Out-Null
    }

    $user = (Invoke-Docker @('image', 'inspect', '--format', '{{.Config.User}}', $Image)).Output
    if ($user -ne '10001:10001') { throw "the image must run as 10001:10001, it runs as '$user'" }

    Write-Step "docker run -d -p ${HostPort}:7777/udp"
    Invoke-Docker @('rm', '-f', $Name) -AllowFailure | Out-Null
    Invoke-Docker @('run', '-d', '--name', $Name, '-p', "${HostPort}:7777/udp", $Image) | Out-Null
    $containerStarted = $true

    Write-Step 'waiting for the relay to answer --selfcheck'
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    $healthy = $false
    while ([DateTime]::UtcNow -lt $deadline) {
        $state = (Invoke-Docker @('inspect', '--format', '{{.State.Running}}', $Name)).Output
        if ($state -ne 'true') {
            $logs = (Invoke-Docker @('logs', $Name) -AllowFailure).Output
            throw "the relay container stopped unexpectedly:`n$logs"
        }
        if ((Invoke-Docker @('exec', $Name, 'wpiemu_relay', '--selfcheck') -AllowFailure).ExitCode -eq 0) {
            $healthy = $true
            break
        }
        Start-Sleep -Seconds 1
    }
    if (-not $healthy) { throw "the relay did not pass --selfcheck within $TimeoutSeconds s" }
    Write-Host 'selfcheck ok'

    $logs = (Invoke-Docker @('logs', $Name)).Output
    if ($logs -notmatch 'starting') { throw "the relay log has no start line:`n$logs" }

    # The round-trip probe is written separately; run it only when this image's probe knows --relay.
    $help = (Invoke-Docker @('exec', $Name, 'wpiemu_relay_probe', '--help') -AllowFailure).Output
    if ($help -match '--relay') {
        Write-Step "wpiemu_relay_probe --relay 127.0.0.1:7777 (inside the container)"
        Invoke-Docker @('exec', $Name, 'wpiemu_relay_probe', '--relay', '127.0.0.1:7777') | Out-Null
        Write-Host 'probe ok'
    } else {
        Write-Host 'probe skipped: wpiemu_relay_probe has no --relay option yet'
    }

    Write-Step 'docker stop (SIGTERM) and exit code'
    Invoke-Docker @('stop', '--time', '15', $Name) | Out-Null
    $exitCode = (Invoke-Docker @('inspect', '--format', '{{.State.ExitCode}}', $Name)).Output
    if ($exitCode -ne '0') {
        $logs = (Invoke-Docker @('logs', $Name) -AllowFailure).Output
        throw "the relay exited with code $exitCode after SIGTERM (expected 0):`n$logs"
    }
    $logs = (Invoke-Docker @('logs', $Name)).Output
    if ($logs -notmatch 'stopped') { throw "the relay log has no 'stopped' line after SIGTERM:`n$logs" }
    Write-Host 'container smoke test passed'
}
catch {
    $failure = $_
    if ($containerStarted) {
        Write-Host '--- container logs ---'
        (Invoke-Docker @('logs', '--tail', '50', $Name) -AllowFailure).Output | Write-Host
    }
}
finally {
    if (Get-Command docker -ErrorAction SilentlyContinue) {
        Invoke-Docker @('rm', '-f', $Name) -AllowFailure | Out-Null
        if (-not $KeepImage -and -not $SkipBuild) { Invoke-Docker @('rmi', '-f', $Image) -AllowFailure | Out-Null }
    }
}
if ($failure) {
    [Console]::Error.WriteLine("container smoke test FAILED: $($failure.Exception.Message)")
    exit 1
}
