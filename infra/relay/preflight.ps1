#Requires -Version 7.0
<#
.SYNOPSIS
Read-only Azure region, provider, quota and tooling checks for the WPI Emulator relay.

.DESCRIPTION
Verifies with the Azure CLI that the relay (ACI container group, ACR, Log Analytics, managed identity)
can be deployed to -Location. It never logs in and never creates or modifies resources. The only
mutating calls are opt-in: -Register runs `az provider register` and -InstallBicep runs `az bicep install`.

Checks (each reports PASS, FAIL or WARN, and FAIL carries a remediation line):
  1. account        az is present, you are logged in and the subscription is Enabled.
  2. location       the location exists for the subscription (expect "South Central US").
  3. provider       the required resource providers are Registered.
  4. region         each resource type lists the location (names compared without spaces, lower-case).
  5. quota          container group and core headroom in the region (ACI usages API through az rest),
                    and -UdpPorts within the documented limit of 5 ports per IP.
  6. bicep          `az bicep version` works.

When a region-scoped check fails, the nearest regions that pass the region and quota checks are listed, in
this preference order: southcentralus, centralus, eastus2, eastus, westus2, westus3, northcentralus.

.PARAMETER Location
Azure region name to verify. Default: southcentralus.

.PARAMETER SubscriptionId
Optional subscription id or name, passed to az with --subscription.

.PARAMETER Register
Register unregistered providers (`az provider register`). Without it the exact command is printed.

.PARAMETER InstallBicep
Run `az bicep install` when Bicep is missing. Without it the command is printed.

.PARAMETER Json
Print one machine-readable JSON document on stdout instead of the text report.

.PARAMETER RequiredCpu
vCPUs the relay needs (default 0.5).

.PARAMETER RequiredMemoryGb
Memory in GB the relay needs (default 1).

.PARAMETER UdpPorts
Number of UDP ports the relay exposes on its IP (default 1, limit 5).

.OUTPUTS
Text report, or JSON with -Json.

Exit codes:
  0  every check passed (warnings are allowed)
  1  at least one check failed
  2  az is missing or you are not logged in (run `az login` yourself, this script never does)

.EXAMPLE
pwsh -NoProfile -File infra/relay/preflight.ps1

.EXAMPLE
pwsh -NoProfile -File infra/relay/preflight.ps1 -Location centralus -Json
#>
[CmdletBinding()]
param(
    [ValidatePattern('^[a-z0-9]+$')]
    [string]$Location = 'southcentralus',
    [string]$SubscriptionId = '',
    [switch]$Register,
    [switch]$InstallBicep,
    [switch]$Json,
    [ValidateRange(0.01, 1000)]
    [double]$RequiredCpu = 0.5,
    [ValidateRange(0.01, 10000)]
    [double]$RequiredMemoryGb = 1,
    [ValidateRange(1, 1000)]
    [int]$UdpPorts = 1
)

Set-StrictMode -Version Latest

$script:RequiredProviders = @(
    'Microsoft.ContainerInstance',
    'Microsoft.ContainerRegistry',
    'Microsoft.OperationalInsights',
    'Microsoft.Consumption',
    'Microsoft.ManagedIdentity'
)
$script:ResourceTypes = @(
    @{ Namespace = 'Microsoft.ContainerInstance'; Type = 'containerGroups' },
    @{ Namespace = 'Microsoft.ContainerRegistry'; Type = 'registries' },
    @{ Namespace = 'Microsoft.OperationalInsights'; Type = 'workspaces' },
    @{ Namespace = 'Microsoft.ManagedIdentity'; Type = 'userAssignedIdentities' }
)
$script:RegionPreference = @('southcentralus', 'centralus', 'eastus2', 'eastus', 'westus2', 'westus3', 'northcentralus')
$script:DefaultGroupLimit = 100
$script:DefaultCoreLimit = 100
$script:MaxUdpPortsPerIp = 5

# Tests replace this with a scriptblock taking ([string[]]$AzArgs) and returning an object with ExitCode and Output.
$script:AzRunner = $null

function Invoke-Az {
    param([Parameter(Mandatory)][string[]]$AzArgs)

    if ($null -ne $script:AzRunner) {
        return & $script:AzRunner $AzArgs
    }
    $all = & az @AzArgs --only-show-errors 2>&1
    $code = $LASTEXITCODE
    $out = @($all | Where-Object { $_ -isnot [System.Management.Automation.ErrorRecord] })
    $err = @($all | Where-Object { $_ -is [System.Management.Automation.ErrorRecord] } | ForEach-Object { $_.ToString() })
    [pscustomobject]@{ ExitCode = $code; Output = (($out + $err) -join "`n"); StdOut = ($out -join "`n") }
}

function Test-AzAuthFailure {
    param([string]$Text)
    $Text -match 'az login|AADSTS|InteractionRequired|not logged in|No subscription found|Please log in'
}

function Invoke-AzJson {
    param([Parameter(Mandatory)][string[]]$AzArgs)

    $r = Invoke-Az -AzArgs $AzArgs
    if ($r.ExitCode -ne 0) {
        if (Test-AzAuthFailure $r.Output) {
            throw [System.InvalidOperationException]::new("AZ_AUTH: $($r.Output.Trim())")
        }
        return [pscustomobject]@{ Ok = $false; Data = $null; Error = $r.Output.Trim() }
    }
    $text = if ($r.PSObject.Properties['StdOut']) { $r.StdOut } else { $r.Output }
    $data = $null
    if (-not [string]::IsNullOrWhiteSpace($text)) {
        try { $data = $text | ConvertFrom-Json -NoEnumerate -ErrorAction Stop }
        catch { return [pscustomobject]@{ Ok = $false; Data = $null; Error = "unparseable az output: $text" } }
    }
    [pscustomobject]@{ Ok = $true; Data = $data; Error = '' }
}

function ConvertTo-RegionKey {
    param([AllowNull()][string]$Name)
    if ($null -eq $Name) { return '' }
    ($Name -replace '\s', '').ToLowerInvariant()
}

function Get-Prop {
    param($Object, [string]$Name, $Default = $null)
    if ($null -ne $Object -and $Object.PSObject.Properties[$Name]) { return $Object.$Name }
    $Default
}

function New-Check {
    param([string]$Id, [string]$Status, [string]$Message, [string]$Remediation = '')
    [pscustomobject]@{ Id = $Id; Status = $Status; Message = $Message; Remediation = $Remediation }
}

function Get-SubArgs {
    param([string]$SubscriptionId)
    if ([string]::IsNullOrWhiteSpace($SubscriptionId)) { return @() }
    @('--subscription', $SubscriptionId)
}

function Test-AzAccount {
    param([string]$SubscriptionId)

    $r = Invoke-AzJson -AzArgs (@('account', 'show', '-o', 'json') + (Get-SubArgs $SubscriptionId))
    if (-not $r.Ok -or $null -eq $r.Data) {
        throw [System.InvalidOperationException]::new("AZ_AUTH: $($r.Error)")
    }
    $state = [string](Get-Prop $r.Data 'state' '')
    $name = [string](Get-Prop $r.Data 'name' '')
    $id = [string](Get-Prop $r.Data 'id' '')
    $check = if ($state -eq 'Enabled') {
        New-Check 'account' 'PASS' "Logged in; subscription '$name' ($id) is Enabled."
    }
    else {
        New-Check 'account' 'FAIL' "Subscription '$name' ($id) state is '$state', not Enabled." `
            'Select an enabled subscription: az account set --subscription <id>'
    }
    [pscustomobject]@{ Check = $check; SubscriptionId = $id }
}

function Test-AzLocation {
    param([string]$Location, [string]$SubscriptionId)

    $query = "[?name=='$Location'].displayName"
    $r = Invoke-AzJson -AzArgs (@('account', 'list-locations', '--query', $query, '-o', 'json') + (Get-SubArgs $SubscriptionId))
    if (-not $r.Ok) {
        return New-Check 'location' 'FAIL' "Could not list locations: $($r.Error)" 'Check az connectivity and permissions.'
    }
    $names = @(@($r.Data) | Where-Object { $null -ne $_ })
    if ($names.Count -eq 0) {
        return New-Check 'location' 'FAIL' "Location '$Location' is not available to this subscription." `
            'Pick a region from: az account list-locations --query "[].name" -o tsv'
    }
    New-Check 'location' 'PASS' "Location '$Location' exists (display name '$($names[0])')."
}

function Test-AzProviders {
    param([string]$SubscriptionId, [switch]$Register)

    foreach ($ns in $script:RequiredProviders) {
        $r = Invoke-AzJson -AzArgs (@('provider', 'show', '--namespace', $ns, '--query', 'registrationState', '-o', 'json') + (Get-SubArgs $SubscriptionId))
        $id = "provider:$ns"
        if (-not $r.Ok) {
            New-Check $id 'FAIL' "Could not read registration state of ${ns}: $($r.Error)" "az provider show --namespace $ns"
            continue
        }
        $state = [string]$r.Data
        $cmd = "az provider register --namespace $ns" + $(if ($SubscriptionId) { " --subscription $SubscriptionId" } else { '' })
        if ($state -eq 'Registered') {
            New-Check $id 'PASS' "$ns is Registered."
        }
        elseif ($state -eq 'Registering') {
            New-Check $id 'WARN' "$ns is still Registering; re-run later."
        }
        elseif ($Register) {
            $reg = Invoke-Az -AzArgs (@('provider', 'register', '--namespace', $ns) + (Get-SubArgs $SubscriptionId))
            if ($reg.ExitCode -eq 0) {
                New-Check $id 'WARN' "$ns was '$state'; registration requested, re-run when it is Registered."
            }
            else {
                New-Check $id 'FAIL' "$ns is '$state' and registration failed: $($reg.Output.Trim())" $cmd
            }
        }
        else {
            New-Check $id 'FAIL' "$ns is '$state', not Registered." "$cmd (or re-run with -Register)"
        }
    }
}

function Get-ResourceTypeRegions {
    param([string]$Namespace, [string]$Type, [string]$SubscriptionId)

    $query = "resourceTypes[?resourceType=='$Type'].locations | [0]"
    $r = Invoke-AzJson -AzArgs (@('provider', 'show', '--namespace', $Namespace, '--query', $query, '-o', 'json') + (Get-SubArgs $SubscriptionId))
    if (-not $r.Ok) { return [pscustomobject]@{ Ok = $false; Regions = @(); Error = $r.Error } }
    $keys = @(@($r.Data) | Where-Object { $null -ne $_ } | ForEach-Object { ConvertTo-RegionKey $_ })
    [pscustomobject]@{ Ok = $true; Regions = $keys; Error = '' }
}

function Get-QuotaStatus {
    param([string]$Location, [string]$ResolvedSubscriptionId, [double]$RequiredCpu, [double]$RequiredMemoryGb)

    $problems = @()
    $notes = @()
    $url = "https://management.azure.com/subscriptions/$ResolvedSubscriptionId/providers/Microsoft.ContainerInstance/locations/$Location/usages?api-version=2023-05-01"
    $r = Invoke-AzJson -AzArgs @('rest', '--method', 'get', '--url', $url, '-o', 'json')
    $usageOk = $r.Ok -and $null -ne $r.Data
    $items = @()
    if ($usageOk) { $items = @(@(Get-Prop $r.Data 'value' @()) | Where-Object { $null -ne $_ }) }

    $find = {
        param([string]$Pattern)
        $items | Where-Object {
            $n = Get-Prop $_ 'name' $null
            (ConvertTo-RegionKey ([string](Get-Prop $n 'value' ''))) -match $Pattern -or
            (ConvertTo-RegionKey ([string](Get-Prop $n 'localizedValue' ''))) -match $Pattern
        } | Select-Object -First 1
    }

    $limits = @(
        @{ Label = 'container groups'; Pattern = '^containergroups$'; Need = 1.0; Default = $script:DefaultGroupLimit },
        @{ Label = 'cores'; Pattern = '^(standard)?cores$'; Need = $RequiredCpu; Default = $script:DefaultCoreLimit }
    )
    foreach ($l in $limits) {
        $item = $null
        if ($usageOk) { $item = & $find $l.Pattern }
        $limit = [double]$l.Default
        $current = 0.0
        if ($null -ne $item) {
            $lv = Get-Prop $item 'limit' $null
            $cv = Get-Prop $item 'currentValue' $null
            if ($null -ne $lv) { $limit = [double]$lv } else { $notes += "no limit reported for $($l.Label), assumed $($l.Default)" }
            if ($null -ne $cv) { $current = [double]$cv }
        }
        else {
            $notes += "no usage entry for $($l.Label), assumed default limit $($l.Default)"
        }
        if (($limit - $current) -lt $l.Need) {
            $problems += "$($l.Label): $current of $limit used, need $($l.Need) more"
        }
    }

    $mem = $null
    if ($usageOk) { $mem = & $find 'memory' }
    if ($null -ne $mem) {
        $lv = Get-Prop $mem 'limit' $null
        if ($null -ne $lv -and (([double]$lv - [double](Get-Prop $mem 'currentValue' 0)) -lt $RequiredMemoryGb)) {
            $problems += "memory: need $RequiredMemoryGb GB more than available"
        }
    }

    if (-not $usageOk) { $notes += "usage could not be read ($($r.Error)), documented defaults assumed" }
    [pscustomobject]@{ Problems = $problems; Notes = $notes }
}

function Test-AzQuota {
    param([string]$Location, [string]$ResolvedSubscriptionId, [double]$RequiredCpu, [double]$RequiredMemoryGb, [int]$UdpPorts)

    $checks = @()
    $q = Get-QuotaStatus -Location $Location -ResolvedSubscriptionId $ResolvedSubscriptionId -RequiredCpu $RequiredCpu -RequiredMemoryGb $RequiredMemoryGb
    if ($q.Problems.Count -gt 0) {
        $checks += New-Check 'quota' 'FAIL' ("Insufficient ACI headroom in ${Location}: " + ($q.Problems -join '; ') + '.') `
            'Request more quota (Azure portal > Quotas > Container Instances) or choose another region.'
    }
    elseif ($q.Notes.Count -gt 0) {
        $checks += New-Check 'quota' 'WARN' ("Headroom OK for $RequiredCpu vCPU / $RequiredMemoryGb GB in ${Location}, with caveats: " + ($q.Notes -join '; ') + '.')
    }
    else {
        $checks += New-Check 'quota' 'PASS' "Headroom for 1 container group, $RequiredCpu vCPU and $RequiredMemoryGb GB in $Location."
    }

    if ($UdpPorts -gt $script:MaxUdpPortsPerIp) {
        $checks += New-Check 'udp-ports' 'FAIL' "$UdpPorts UDP ports requested, the documented limit is $($script:MaxUdpPortsPerIp) ports per IP." `
            "Expose at most $($script:MaxUdpPortsPerIp) ports on the container group."
    }
    else {
        $checks += New-Check 'udp-ports' 'PASS' "$UdpPorts UDP port(s) requested, within the limit of $($script:MaxUdpPortsPerIp) per IP."
    }
    $checks
}

function Test-AzBicep {
    param([switch]$InstallBicep)

    $r = Invoke-Az -AzArgs @('bicep', 'version')
    if ($r.ExitCode -ne 0 -and $InstallBicep) {
        $null = Invoke-Az -AzArgs @('bicep', 'install')
        $r = Invoke-Az -AzArgs @('bicep', 'version')
    }
    if ($r.ExitCode -eq 0) {
        return New-Check 'bicep' 'PASS' "Bicep available: $($r.Output.Trim())"
    }
    New-Check 'bicep' 'FAIL' 'Bicep CLI is not installed.' 'az bicep install (or re-run with -InstallBicep)'
}

function Invoke-Preflight {
    [CmdletBinding()]
    param(
        [string]$Location = 'southcentralus',
        [string]$SubscriptionId = '',
        [switch]$Register,
        [switch]$InstallBicep,
        [double]$RequiredCpu = 0.5,
        [double]$RequiredMemoryGb = 1,
        [int]$UdpPorts = 1
    )

    $checks = [System.Collections.Generic.List[object]]::new()
    $suggested = [System.Collections.Generic.List[string]]::new()
    $newResult = {
        param([int]$Code)
        [pscustomobject]@{
            Location         = $Location
            SubscriptionId   = $SubscriptionId
            ExitCode         = $Code
            Checks           = $checks.ToArray()
            SuggestedRegions = $suggested.ToArray()
        }
    }

    if ($null -eq $script:AzRunner -and -not (Get-Command az -ErrorAction SilentlyContinue)) {
        $checks.Add((New-Check 'account' 'FAIL' 'The Azure CLI (az) was not found on PATH.' 'Install the Azure CLI (https://aka.ms/installazurecli), then run az login.'))
        return & $newResult 2
    }

    try {
        $acct = Test-AzAccount -SubscriptionId $SubscriptionId
        $checks.Add($acct.Check)
        $subId = if ($SubscriptionId) { $SubscriptionId } else { $acct.SubscriptionId }

        $locationCheck = Test-AzLocation -Location $Location -SubscriptionId $SubscriptionId
        $checks.Add($locationCheck)

        foreach ($c in @(Test-AzProviders -SubscriptionId $SubscriptionId -Register:$Register)) { $checks.Add($c) }

        $regionSets = @{}
        $supportFailed = $false
        foreach ($rt in $script:ResourceTypes) {
            $id = "region:$($rt.Namespace)/$($rt.Type)"
            $r = Get-ResourceTypeRegions -Namespace $rt.Namespace -Type $rt.Type -SubscriptionId $SubscriptionId
            $regionSets[$id] = $r.Regions
            if (-not $r.Ok) {
                $checks.Add((New-Check $id 'FAIL' "Could not read supported regions: $($r.Error)" "az provider show --namespace $($rt.Namespace)"))
                $supportFailed = $true
            }
            elseif ($r.Regions.Count -eq 0) {
                $checks.Add((New-Check $id 'FAIL' "$($rt.Namespace)/$($rt.Type) was not found or lists no regions." "az provider show --namespace $($rt.Namespace)"))
                $supportFailed = $true
            }
            elseif ($r.Regions -contains (ConvertTo-RegionKey $Location)) {
                $checks.Add((New-Check $id 'PASS' "$($rt.Namespace)/$($rt.Type) is supported in $Location."))
            }
            else {
                $checks.Add((New-Check $id 'FAIL' "$($rt.Namespace)/$($rt.Type) does not list $Location." 'Change the location parameter to one of the suggested regions.'))
                $supportFailed = $true
            }
        }

        $quotaChecks = @(Test-AzQuota -Location $Location -ResolvedSubscriptionId $subId -RequiredCpu $RequiredCpu -RequiredMemoryGb $RequiredMemoryGb -UdpPorts $UdpPorts)
        foreach ($c in $quotaChecks) { $checks.Add($c) }

        $checks.Add((Test-AzBicep -InstallBicep:$InstallBicep))

        $quotaFailed = @($quotaChecks | Where-Object { $_.Id -eq 'quota' -and $_.Status -eq 'FAIL' }).Count -gt 0
        if ($supportFailed -or $locationCheck.Status -eq 'FAIL' -or $quotaFailed) {
            $wanted = ConvertTo-RegionKey $Location
            foreach ($candidate in $script:RegionPreference) {
                if ($candidate -eq $wanted) { continue }
                $passesAll = $true
                foreach ($set in $regionSets.Values) { if ($set -notcontains $candidate) { $passesAll = $false } }
                if (-not $passesAll) { continue }
                $cq = Get-QuotaStatus -Location $candidate -ResolvedSubscriptionId $subId -RequiredCpu $RequiredCpu -RequiredMemoryGb $RequiredMemoryGb
                if ($cq.Problems.Count -eq 0) { $suggested.Add($candidate) }
            }
            $suggestionText = if ($suggested.Count -gt 0) { "Nearest passing regions: $($suggested -join ', ')." } else { 'No region from the preference list passes every region check.' }
            $checks.Add((New-Check 'suggestion' 'WARN' $suggestionText))
        }
    }
    catch [System.InvalidOperationException] {
        if ($_.Exception.Message -notlike 'AZ_AUTH:*') { throw }
        $detail = $_.Exception.Message.Substring('AZ_AUTH:'.Length).Trim()
        $checks.Add((New-Check 'login' 'FAIL' "Not logged in to Azure or the session expired. $detail" 'Run az login yourself (this script never logs in), then re-run.'))
        return & $newResult 2
    }

    $failed = @($checks | Where-Object { $_.Status -eq 'FAIL' }).Count
    return & $newResult $(if ($failed -gt 0) { 1 } else { 0 })
}

function Write-PreflightReport {
    param([Parameter(Mandatory)]$Result)

    $colors = @{ PASS = 'Green'; FAIL = 'Red'; WARN = 'Yellow' }
    Write-Host "Relay preflight for '$($Result.Location)'"
    foreach ($c in $Result.Checks) {
        Write-Host ('[{0}] {1}: {2}' -f $c.Status, $c.Id, $c.Message) -ForegroundColor $colors[$c.Status]
        if ($c.Remediation) { Write-Host "       fix: $($c.Remediation)" }
    }
    $counts = foreach ($s in 'PASS', 'WARN', 'FAIL') { "$(@($Result.Checks | Where-Object { $_.Status -eq $s }).Count) $s" }
    Write-Host ('Result: {0} (exit {1})' -f ($counts -join ', '), $Result.ExitCode)
}

if ($MyInvocation.InvocationName -ne '.') {
    $outcome = Invoke-Preflight -Location $Location -SubscriptionId $SubscriptionId -Register:$Register `
        -InstallBicep:$InstallBicep -RequiredCpu $RequiredCpu -RequiredMemoryGb $RequiredMemoryGb -UdpPorts $UdpPorts
    if ($Json) {
        $outcome | ConvertTo-Json -Depth 6
    }
    else {
        Write-PreflightReport -Result $outcome
    }
    exit $outcome.ExitCode
}
