<#
.SYNOPSIS
  Manual end-to-end test: real WPILib Java project + real halsim_wpiemu + fake emulator over the real pipe.
.DESCRIPTION
  Copies the project to $env:TEMP (the original is never modified or built in), starts
  wpiemu_halsim_fake_emulator, runs `gradlew -I wpiemu-init.gradle simulateJava`, waits for the
  emulator's SUMMARY line, kills the robot by PID, and deletes the copy.
  Not part of ctest (needs a WPILib installation, Gradle and a minute of wall time).
.EXAMPLE
  .\run_robot_project.ps1 -Project C:\Users\me\FRC-Robot -BuildDir C:\src\wpi-emulator\build\windows-ninja -Config Release
#>
param(
    [Parameter(Mandatory)] [string] $Project,
    [Parameter(Mandatory)] [string] $BuildDir,
    [string] $Config = 'Release',
    [string] $JavaHome = 'C:\Users\Public\wpilib\2026\jdk',
    [double] $Seconds = 5,
    [int] $TimeoutSeconds = 600,
    [switch] $KeepCopy
)
$ErrorActionPreference = 'Stop'
$bin = Join-Path $BuildDir "bin\$Config"
$ext = Join-Path $bin 'halsim_wpiemu.dll'
$emu = Join-Path $bin 'wpiemu_halsim_fake_emulator.exe'
foreach ($f in $ext, $emu) { if (-not (Test-Path $f)) { throw "missing $f (build halsim_wpiemu and wpiemu_halsim_fake_emulator)" } }

$work = Join-Path $env:TEMP ("wpiemu-e2e-" + [guid]::NewGuid().ToString('N').Substring(0, 8))
$copy = Join-Path $work 'project'
New-Item -ItemType Directory -Force $copy | Out-Null
# Never touch the original: copy without VCS/build state, so Gradle rebuilds in the copy.
robocopy $Project $copy /E /XD .git build .gradle logs /NFL /NDL /NJH /NJS /NP | Out-Null

$emuOut = Join-Path $work 'emulator.out'
$emuErr = Join-Path $work 'emulator.err'
$endpointFile = Join-Path $work 'endpoint.txt'
$robotProc = $null
$emuProc = Start-Process -FilePath $emu -ArgumentList @('--endpoint-file', $endpointFile, '--seconds', $Seconds, '--dump-outputs') `
    -RedirectStandardOutput $emuOut -RedirectStandardError $emuErr -PassThru -WindowStyle Hidden
try {
    for ($i = 0; $i -lt 100 -and -not (Test-Path $endpointFile); $i++) { Start-Sleep -Milliseconds 100 }
    if (-not (Test-Path $endpointFile)) { throw 'fake emulator did not publish its endpoint' }

    $env:WPIEMU_IPC_ENDPOINT = (Get-Content $endpointFile -Raw).Trim()
    $env:WPIEMU_EXT = $ext
    $env:JAVA_HOME = $JavaHome
    $env:WPIEMU_LOG_LEVEL = 'debug'
    $gradleOut = Join-Path $work 'gradle.out'
    $gradleErr = Join-Path $work 'gradle.err'
    $gradle = Start-Process -FilePath (Join-Path $copy 'gradlew.bat') `
        -ArgumentList @('--no-daemon', '--console=plain', '-I', (Join-Path $PSScriptRoot 'wpiemu-init.gradle'), 'simulateJava') `
        -WorkingDirectory $copy -RedirectStandardOutput $gradleOut -RedirectStandardError $gradleErr -PassThru -WindowStyle Hidden

    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    while ((Get-Date) -lt $deadline -and -not $emuProc.HasExited) { Start-Sleep -Milliseconds 500 }
    if (-not $emuProc.HasExited) { throw "fake emulator did not finish within $TimeoutSeconds s" }

    Write-Host '--- fake emulator ---'
    Get-Content $emuOut | Select-Object -First 80
    if ((Get-Content $emuErr -ErrorAction SilentlyContinue)) { Write-Host '--- emulator stderr ---'; Get-Content $emuErr }
    Write-Host '--- robot/gradle (extension lines) ---'
    Get-Content $gradleOut, $gradleErr -ErrorAction SilentlyContinue | Select-String -Pattern 'wpiemu|HAL Extensions|startup complete|ERROR'

    $hello = (Get-Content $emuOut | Select-String -Pattern '^HELLO pid=(\d+)')
    if ($hello) { $robotProc = [int]$hello.Matches[0].Groups[1].Value }
    if ($emuProc.ExitCode -ne 0) { throw "fake emulator exit code $($emuProc.ExitCode)" }
    Write-Host 'E2E OK'
}
finally {
    # By PID only: the robot JVM from the emulator's Hello, then the Gradle client we started.
    if ($robotProc) { Stop-Process -Id $robotProc -Force -ErrorAction SilentlyContinue }
    if ($gradle -and -not $gradle.HasExited) { Start-Sleep -Seconds 3; Stop-Process -Id $gradle.Id -Force -ErrorAction SilentlyContinue }
    if ($emuProc -and -not $emuProc.HasExited) { Stop-Process -Id $emuProc.Id -Force -ErrorAction SilentlyContinue }
    if (-not $KeepCopy) { Remove-Item -Recurse -Force $work -ErrorAction SilentlyContinue } else { Write-Host "kept $work" }
}
