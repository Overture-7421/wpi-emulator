<#
.SYNOPSIS
    Runs the practice-match end-to-end test (e2e.practice_match) under packet loss, latency and reordering on Windows.

.DESCRIPTION
    The test already includes an in-process impaired network (ImpairedTransport uplink with loss and latency plus a delayed
    downlink for one player) that runs whenever the e2e is enabled. This script adds the REAL network path: it starts clumsy
    (WinDivert based, https://jagt.github.io/clumsy/) with a filter that matches only the relay's UDP port on loopback, runs
    the e2e against the in-process relay bound to that fixed port, and stops clumsy again.

    What is automatic and what is manual:
      * Automatic: the ImpairedTransport player (every run), this script's clumsy start, run and stop.
      * Manual, once: install clumsy (it is not in this repository and not installed by any script) and run PowerShell elevated;
        the WinDivert driver needs administrator rights. Some antivirus products flag the driver.
      * Manual, judgement: whether a failure under impairment is a defect. Distance and timing checks of the test are written for
        a LAN. Under -Profile bad treat a failing position check as a finding to look at, not as proof of a defect.
      * Not realistic on Windows: Linux "tc netem" shapes Linux interfaces only. To use it, run the relay in WSL2 or a container
        with NET_ADMIN, apply `tc qdisc add dev eth0 root netem delay 40ms 10ms loss 3%` there, and point the test at it with
        WPIEMU_E2E_RELAY=<address>:<port> (the checks that read the relay's counters are then skipped).
      * Not usable for loss or latency: New-NetQosPolicy only throttles bandwidth (and ignores loopback). It can emulate a thin
        uplink toward a deployed relay: New-NetQosPolicy -Name wpiemu-e2e -IPProtocolMatchCondition UDP
        -IPDstPortStartMatchCondition <port> -IPDstPortEndMatchCondition <port> -ThrottleRateActionBitsPerSecond 256000
        (elevated; remove it with Remove-NetQosPolicy -Name wpiemu-e2e). Run the e2e with WPIEMU_E2E_RELAY pointing at the
        deployed relay while it is active.

    Loopback traffic is seen by WinDivert once, as outbound, so every impairment applies to each direction independently: a lag of
    40 ms means about 80 ms round trip. The test prints the round trip it measured for the impaired player.

.PARAMETER Profile
    lan     no impairment (baseline run through the same path)
    wan     40 ms lag, 2 % drop
    lossy   20 ms lag, 8 % drop, 3 % reordered
    bad     80 ms lag, 10 % drop, 5 % reordered, 2 % duplicated
    custom  only the explicit -LagMs, -DropPercent, -ReorderPercent and -DuplicatePercent

.PARAMETER Clumsy
    Path to clumsy.exe (default: found on PATH).

.PARAMETER ExtraClumsyArgs
    Extra command line arguments for clumsy. Option names differ between clumsy versions: check `clumsy.exe --help`.

.PARAMETER DryRun
    Print the filter, the clumsy command line and the test command, and run nothing.

.EXAMPLE
    # elevated PowerShell, from the repository root, after building wpiemu_e2e_match_tests (Release)
    pwsh tools/e2e_netem.ps1 -Profile wan
    pwsh tools/e2e_netem.ps1 -Profile custom -LagMs 100 -DropPercent 5 -DryRun
#>
[CmdletBinding()]
param(
    [ValidateSet('lan', 'wan', 'lossy', 'bad', 'custom')][string]$Profile = 'wan',
    [int]$LagMs = 0,
    [double]$DropPercent = 0,
    [double]$ReorderPercent = 0,
    [double]$DuplicatePercent = 0,
    [ValidateRange(1024, 65535)][int]$RelayPort = 47777,
    [string]$Clumsy = 'clumsy.exe',
    [string[]]$ExtraClumsyArgs = @(),
    [string]$BuildDir = (Join-Path $PSScriptRoot '..\build'),
    [string]$Config = 'Release',
    [string]$Project = '',
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'

switch ($Profile) {
    'lan'   { $LagMs = 0;  $DropPercent = 0;  $ReorderPercent = 0; $DuplicatePercent = 0 }
    'wan'   { $LagMs = 40; $DropPercent = 2;  $ReorderPercent = 0; $DuplicatePercent = 0 }
    'lossy' { $LagMs = 20; $DropPercent = 8;  $ReorderPercent = 3; $DuplicatePercent = 0 }
    'bad'   { $LagMs = 80; $DropPercent = 10; $ReorderPercent = 5; $DuplicatePercent = 2 }
}

# Loopback packets are captured once, as outbound. Only the relay's UDP port is touched; the robot's local IPC is not UDP.
$filter = "outbound and udp and ip.DstAddr >= 127.0.0.1 and ip.DstAddr <= 127.255.255.255 and (udp.DstPort == $RelayPort or udp.SrcPort == $RelayPort)"

$clumsyArgs = @('--filter', ('"{0}"' -f $filter))   # Start-Process joins the arguments with spaces: the filter needs its own quotes
if ($LagMs -gt 0) { $clumsyArgs += @('--lag', 'on', '--lag-time', "$LagMs") }
if ($DropPercent -gt 0) { $clumsyArgs += @('--drop', 'on', '--drop-chance', "$DropPercent") }
if ($ReorderPercent -gt 0) { $clumsyArgs += @('--ood', 'on', '--ood-chance', "$ReorderPercent") }
if ($DuplicatePercent -gt 0) { $clumsyArgs += @('--duplicate', 'on', '--duplicate-count', '1', '--duplicate-chance', "$DuplicatePercent") }
$clumsyArgs += $ExtraClumsyArgs

$exe = Join-Path $BuildDir "bin\$Config\wpiemu_e2e_match_tests.exe"
$testEnvironment = @{ WPIEMU_E2E = '1'; WPIEMU_E2E_RELAY_PORT = "$RelayPort" }
if ($Project) { $testEnvironment['WPIEMU_E2E_PROJECT'] = $Project }

Write-Host "profile : $Profile (lag $LagMs ms, drop $DropPercent %, reorder $ReorderPercent %, duplicate $DuplicatePercent %)"
Write-Host "filter  : $filter"
Write-Host "clumsy  : $Clumsy $($clumsyArgs -join ' ')"
Write-Host "test    : $exe   (WPIEMU_E2E_RELAY_PORT=$RelayPort)"
if ($DryRun) { Write-Host 'dry run: nothing started.'; return }

$principal = [Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'Run this script from an elevated PowerShell: clumsy needs administrator rights for the WinDivert driver.'
}
if ($Profile -ne 'lan' -and -not (Get-Command $Clumsy -ErrorAction SilentlyContinue)) {
    throw "clumsy not found ('$Clumsy'). Install it from https://jagt.github.io/clumsy/ or pass -Clumsy <path>."
}
if (-not (Test-Path $exe)) {
    throw "Build the test first: cmake --build $BuildDir --config $Config --target wpiemu_e2e_match_tests"
}

$impairer = $null
$exitCode = 1
try {
    if ($Profile -ne 'lan') {
        $impairer = Start-Process -FilePath $Clumsy -ArgumentList $clumsyArgs -PassThru -WindowStyle Minimized
        Start-Sleep -Seconds 2
        if ($impairer.HasExited) { throw "clumsy exited at once (code $($impairer.ExitCode)); check its options with '$Clumsy --help'." }
        Write-Host "clumsy running (pid $($impairer.Id))"
    }
    foreach ($name in $testEnvironment.Keys) { Set-Item -Path "Env:$name" -Value $testEnvironment[$name] }
    & $exe
    $exitCode = $LASTEXITCODE
}
finally {
    if ($impairer -and -not $impairer.HasExited) { Stop-Process -Id $impairer.Id }
}
if ($exitCode -eq 77) { Write-Warning 'The e2e was skipped (a prerequisite is missing); see its message above.' }
exit $exitCode
