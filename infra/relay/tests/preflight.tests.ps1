#Requires -Version 7.0
# Plain PowerShell tests for infra/relay/preflight.ps1 (no Pester needed).
# Run: pwsh -NoProfile -File infra/relay/tests/preflight.tests.ps1
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot '..\preflight.ps1')

$script:Failures = 0
$script:Passed = 0

function Assert-True {
    param([bool]$Condition, [string]$Name)
    if ($Condition) { $script:Passed++; Write-Host "  ok   $Name" }
    else { $script:Failures++; Write-Host "  FAIL $Name" -ForegroundColor Red }
}

function Get-Check { param($Result, [string]$Id) @($Result.Checks | Where-Object { $_.Id -eq $Id })[0] }

$allTypeRegions = @('South Central US', 'Central US', 'East US 2', 'East US', 'West US 2', 'West US 3', 'North Central US')

function New-AzOk { param([string]$Text) [pscustomobject]@{ ExitCode = 0; Output = $Text } }
function New-AzFail { param([string]$Text) [pscustomobject]@{ ExitCode = 1; Output = $Text } }

# Each override replaces the canned answer for a command whose joined arguments match the key regex.
function Use-FakeAz {
    param([hashtable]$Override = @{})

    $script:FakeOverrides = $Override
    $script:AzRunner = {
        param([string[]]$AzArgs)
        $cmd = $AzArgs -join ' '
        foreach ($k in $script:FakeOverrides.Keys) {
            if ($cmd -match $k) { return $script:FakeOverrides[$k] }
        }
        switch -Regex ($cmd) {
            '^account show' { return New-AzOk '{"id":"11111111-1111-1111-1111-111111111111","name":"Test Sub","state":"Enabled"}' }
            '^account list-locations' { return New-AzOk '["South Central US"]' }
            '^provider show .*registrationState' { return New-AzOk '"Registered"' }
            '^provider show' { return New-AzOk ($allTypeRegions | ConvertTo-Json -Compress) }
            '^rest ' {
                return New-AzOk (@{ value = @(
                            @{ name = @{ value = 'ContainerGroups'; localizedValue = 'Container Groups' }; currentValue = 3; limit = 100 },
                            @{ name = @{ value = 'StandardCores'; localizedValue = 'Standard Cores' }; currentValue = 4; limit = 100 }) } | ConvertTo-Json -Depth 6 -Compress)
            }
            '^bicep version' { return New-AzOk 'Bicep CLI version 0.30.3' }
            default { return New-AzFail "unexpected az call: $cmd" }
        }
    }
}

function Invoke-Test {
    param([string]$Name, [scriptblock]$Body)
    Write-Host $Name
    try { & $Body }
    catch { $script:Failures++; Write-Host "  FAIL threw: $($_.Exception.Message)" -ForegroundColor Red }
    finally { $script:AzRunner = $null }
}

Invoke-Test 'all checks pass' {
    Use-FakeAz
    $r = Invoke-Preflight
    Assert-True ($r.ExitCode -eq 0) 'exit code 0'
    Assert-True (@($r.Checks | Where-Object { $_.Status -eq 'FAIL' }).Count -eq 0) 'no FAIL'
    Assert-True ((Get-Check $r 'location').Message -match 'South Central US') 'display name reported'
    Assert-True (@($r.Checks | Where-Object { $_.Id -like 'provider:*' }).Count -eq 5) 'five providers checked'
    Assert-True (@($r.Checks | Where-Object { $_.Id -like 'region:*' }).Count -eq 4) 'four resource types checked'
    Assert-True ($r.SuggestedRegions.Count -eq 0) 'no suggestions'
}

Invoke-Test 'not logged in (account show fails)' {
    Use-FakeAz @{ '^account show' = (New-AzFail "ERROR: Please run 'az login' to setup account.") }
    $r = Invoke-Preflight
    Assert-True ($r.ExitCode -eq 2) 'exit code 2'
    Assert-True ((Get-Check $r 'login').Status -eq 'FAIL') 'login FAIL'
    Assert-True ((Get-Check $r 'login').Remediation -match 'az login') 'remediation names az login'
}

Invoke-Test 'expired token mid-run is exit 2' {
    Use-FakeAz @{ '^provider show' = (New-AzFail 'ERROR: Status_InteractionRequired. Please explicitly log in with: az login') }
    $r = Invoke-Preflight
    Assert-True ($r.ExitCode -eq 2) 'exit code 2'
}

Invoke-Test 'subscription not enabled' {
    Use-FakeAz @{ '^account show' = (New-AzOk '{"id":"x","name":"Old","state":"Disabled"}') }
    $r = Invoke-Preflight
    Assert-True ($r.ExitCode -eq 1) 'exit code 1'
    Assert-True ((Get-Check $r 'account').Status -eq 'FAIL') 'account FAIL'
}

Invoke-Test 'location missing' {
    Use-FakeAz @{ '^account list-locations' = (New-AzOk '[]') }
    $r = Invoke-Preflight
    Assert-True ($r.ExitCode -eq 1) 'exit code 1'
    Assert-True ((Get-Check $r 'location').Status -eq 'FAIL') 'location FAIL'
    Assert-True ((Get-Check $r 'location').Remediation -ne '') 'remediation present'
}

Invoke-Test 'provider unregistered prints the command and does not register' {
    $script:Registered = $false
    Use-FakeAz @{ 'namespace Microsoft.Consumption .*registrationState' = (New-AzOk '"NotRegistered"') }
    $r = Invoke-Preflight
    $c = Get-Check $r 'provider:Microsoft.Consumption'
    Assert-True ($r.ExitCode -eq 1) 'exit code 1'
    Assert-True ($c.Status -eq 'FAIL') 'provider FAIL'
    Assert-True ($c.Remediation -match 'az provider register --namespace Microsoft.Consumption') 'exact register command'
}

Invoke-Test 'provider unregistered with -Register registers' {
    $script:RegisterCalls = @()
    Use-FakeAz @{
        'namespace Microsoft.Consumption .*registrationState' = (New-AzOk '"NotRegistered"')
        '^provider register'                                  = (New-AzOk '')
    }
    $inner = $script:AzRunner
    $script:AzRunner = { param([string[]]$AzArgs) if (($AzArgs -join ' ') -match '^provider register') { $script:RegisterCalls += ($AzArgs -join ' ') }; & $inner $AzArgs }
    $r = Invoke-Preflight -Register
    Assert-True ($script:RegisterCalls.Count -eq 1 -and $script:RegisterCalls[0] -match 'Microsoft.Consumption') 'register called once for the unregistered provider'
    Assert-True ((Get-Check $r 'provider:Microsoft.Consumption').Status -eq 'WARN') 'WARN while registering'
    Assert-True ($r.ExitCode -eq 0) 'exit code 0 (warnings allowed)'
}

Invoke-Test 'resource type missing the region suggests nearest passing regions' {
    $without = @('Central US', 'East US 2', 'East US', 'West US 2', 'North Central US')
    $acrOnly = @('South Central US', 'Central US', 'East US', 'West US 2', 'West US 3')
    Use-FakeAz @{
        'namespace Microsoft.ContainerInstance .*containerGroups' = (New-AzOk ($without | ConvertTo-Json -Compress))
        'namespace Microsoft.ContainerRegistry .*registries'      = (New-AzOk ($acrOnly | ConvertTo-Json -Compress))
    }
    $r = Invoke-Preflight
    Assert-True ($r.ExitCode -eq 1) 'exit code 1'
    Assert-True ((Get-Check $r 'region:Microsoft.ContainerInstance/containerGroups').Status -eq 'FAIL') 'ACI region FAIL'
    Assert-True ((Get-Check $r 'region:Microsoft.ContainerRegistry/registries').Status -eq 'PASS') 'ACR region PASS'
    Assert-True (($r.SuggestedRegions -join ',') -eq 'centralus,eastus,westus2') 'suggestions are the preference order intersected with all passing'
    Assert-True ((Get-Check $r 'suggestion').Message -match 'centralus, eastus, westus2') 'suggestion message'
}

Invoke-Test 'resource type absent entirely' {
    Use-FakeAz @{ 'namespace Microsoft.OperationalInsights .*workspaces' = (New-AzOk 'null') }
    $r = Invoke-Preflight
    Assert-True ((Get-Check $r 'region:Microsoft.OperationalInsights/workspaces').Status -eq 'FAIL') 'workspaces FAIL'
    Assert-True ($r.ExitCode -eq 1) 'exit code 1'
}

Invoke-Test 'quota exhausted' {
    $usage = @{ value = @(
            @{ name = @{ value = 'ContainerGroups' }; currentValue = 100; limit = 100 },
            @{ name = @{ value = 'StandardCores' }; currentValue = 10; limit = 100 }) } | ConvertTo-Json -Depth 6 -Compress
    Use-FakeAz @{ '^rest .*southcentralus' = (New-AzOk $usage) }
    $r = Invoke-Preflight
    Assert-True ($r.ExitCode -eq 1) 'exit code 1'
    Assert-True ((Get-Check $r 'quota').Status -eq 'FAIL') 'quota FAIL'
    Assert-True ((Get-Check $r 'quota').Message -match 'container groups') 'message names container groups'
    Assert-True ($r.SuggestedRegions -contains 'centralus' -and $r.SuggestedRegions -notcontains 'southcentralus') 'other regions suggested'
}

Invoke-Test 'core headroom below the required vCPU' {
    $usage = @{ value = @(
            @{ name = @{ value = 'ContainerGroups' }; currentValue = 0; limit = 100 },
            @{ name = @{ value = 'StandardCores' }; currentValue = 99.8; limit = 100 }) } | ConvertTo-Json -Depth 6 -Compress
    Use-FakeAz @{ '^rest ' = (New-AzOk $usage) }
    $r = Invoke-Preflight -RequiredCpu 0.5
    Assert-True ((Get-Check $r 'quota').Status -eq 'FAIL') 'quota FAIL'
}

Invoke-Test 'missing usage fields fall back to documented defaults with a WARN' {
    Use-FakeAz @{ '^rest ' = (New-AzOk '{"value":[{"name":{"value":"ContainerGroups"},"currentValue":1}]}') }
    $r = Invoke-Preflight
    Assert-True ((Get-Check $r 'quota').Status -eq 'WARN') 'quota WARN'
    Assert-True ($r.ExitCode -eq 0) 'exit code 0'
}

Invoke-Test 'UDP port limit' {
    Use-FakeAz
    $ok = Invoke-Preflight -UdpPorts 5
    $bad = Invoke-Preflight -UdpPorts 6
    Assert-True ((Get-Check $ok 'udp-ports').Status -eq 'PASS') '5 ports pass'
    Assert-True ((Get-Check $bad 'udp-ports').Status -eq 'FAIL' -and $bad.ExitCode -eq 1) '6 ports fail'
}

Invoke-Test 'bicep missing, and installed only with -InstallBicep' {
    $script:Installed = $false
    Use-FakeAz
    $script:Base = $script:AzRunner
    $script:AzRunner = {
        param([string[]]$AzArgs)
        $cmd = $AzArgs -join ' '
        if ($cmd -match '^bicep install') { $script:Installed = $true; return New-AzOk '' }
        if ($cmd -match '^bicep version') { return $(if ($script:Installed) { New-AzOk 'Bicep CLI version 0.30.3' } else { New-AzFail 'Bicep CLI not found' }) }
        & $script:Base $AzArgs
    }
    $r1 = Invoke-Preflight
    Assert-True ((Get-Check $r1 'bicep').Status -eq 'FAIL' -and (Get-Check $r1 'bicep').Remediation -match 'az bicep install') 'FAIL with install command'
    Assert-True (-not $script:Installed) 'did not install by default'
    $r2 = Invoke-Preflight -InstallBicep
    Assert-True ((Get-Check $r2 'bicep').Status -eq 'PASS' -and $script:Installed) 'installed with -InstallBicep'
}

Invoke-Test 'display name and name normalise to the same key' {
    Assert-True ((ConvertTo-RegionKey 'South Central US') -eq (ConvertTo-RegionKey 'southcentralus')) 'South Central US == southcentralus'
    Assert-True ((ConvertTo-RegionKey ' East US 2 ') -eq 'eastus2') 'East US 2 -> eastus2'
    Use-FakeAz @{ 'containerGroups' = (New-AzOk '["southcentralus","centralus"]') }
    $r = Invoke-Preflight
    Assert-True ((Get-Check $r 'region:Microsoft.ContainerInstance/containerGroups').Status -eq 'PASS') 'lower-case provider list matches'
}

Invoke-Test '-SubscriptionId is passed to az' {
    $script:Seen = @()
    Use-FakeAz
    $script:Base = $script:AzRunner
    $script:AzRunner = { param([string[]]$AzArgs) $script:Seen += ($AzArgs -join ' '); & $script:Base $AzArgs }
    $null = Invoke-Preflight -SubscriptionId 'abc'
    Assert-True (@($script:Seen | Where-Object { $_ -match '^account show .*--subscription abc' }).Count -eq 1) 'account show uses --subscription'
    Assert-True (@($script:Seen | Where-Object { $_ -match '^provider show .*--subscription abc' }).Count -gt 0) 'provider show uses --subscription'
}

Write-Host ''
Write-Host "$script:Passed assertions passed, $script:Failures failed"
if ($script:Failures -gt 0) { exit 1 }
exit 0
