#Requires -Version 7.0
# Offline tests for infra/relay (Bicep), no Azure login and no Pester needed. They only run `az bicep build`, `build-params`
# and `lint`, which compile locally, then assert facts on the compiled ARM JSON.
# Run: pwsh -NoProfile -File infra/relay/tests/bicep.tests.ps1
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$script:RelayDir = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$script:RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..')).Path
$script:Scratch = Join-Path $PSScriptRoot '.scratch'
$script:Failures = 0
$script:Passed = 0

function Assert-True {
    param([bool]$Condition, [string]$Name)
    if ($Condition) { $script:Passed++; Write-Host "  ok   $Name" }
    else { $script:Failures++; Write-Host "  FAIL $Name" -ForegroundColor Red }
}

function Assert-Equal {
    param($Actual, $Expected, [string]$Name)
    $same = (($Actual | ConvertTo-Json -Compress -Depth 20) -eq ($Expected | ConvertTo-Json -Compress -Depth 20))
    Assert-True $same $Name
    if (-not $same) { Write-Host "       expected: $($Expected | ConvertTo-Json -Compress -Depth 20)`n       actual:   $($Actual | ConvertTo-Json -Compress -Depth 20)" }
}

# Runs az, returns the exit code, stdout, and the stderr lines. The az "new Bicep release" notice is not a diagnostic.
function Invoke-Az {
    param([string[]]$AzArgs)
    $ErrorActionPreference = 'Continue'
    $all = @(& az @AzArgs 2>&1)
    $code = $LASTEXITCODE
    $stdout = @($all | Where-Object { $_ -isnot [System.Management.Automation.ErrorRecord] }) -join "`n"
    $stderr = @($all | Where-Object { $_ -is [System.Management.Automation.ErrorRecord] } |
            ForEach-Object { $_.ToString() } |
            Where-Object { $_ -notmatch 'new Bicep release is available|az bicep upgrade' })
    [pscustomobject]@{ ExitCode = $code; Stdout = $stdout; Diagnostics = $stderr }
}

function Invoke-Test {
    param([string]$Name, [scriptblock]$Body)
    Write-Host $Name
    try { & $Body }
    catch { $script:Failures++; Write-Host "  FAIL threw: $($_.Exception.Message)" -ForegroundColor Red }
}

# Every resource of the compiled template, including those of nested module templates, with the module it came from.
function Get-ArmResources {
    param($Template, [string]$Module = '', [string]$Condition = '')
    $items = $Template.resources
    if ($null -eq $items) { return }
    if ($items -isnot [array]) { $items = @($items.PSObject.Properties.Value) }
    foreach ($r in $items) {
        if ($r.type -eq 'Microsoft.Resources/deployments' -and $r.properties.PSObject.Properties.Name -contains 'template') {
            $cond = if ($r.PSObject.Properties.Name -contains 'condition') { [string]$r.condition } else { '' }
            Get-ArmResources -Template $r.properties.template -Module $r.name -Condition $cond
        }
        else {
            [pscustomobject]@{ Resource = $r; Module = $Module; ModuleCondition = $Condition }
        }
    }
}

function Get-ModuleDeployments {
    param($Template)
    foreach ($r in @($Template.resources)) { if ($r.type -eq 'Microsoft.Resources/deployments') { $r } }
}

function Get-RelayEnvNames {
    $source = Get-Content -Raw (Join-Path $script:RepoRoot 'relay\src\relay_options.cpp')
    @([regex]::Matches($source, '"(WPIEMU_RELAY_[A-Z0-9_]+)"') | ForEach-Object { $_.Groups[1].Value } | Sort-Object -Unique)
}

function Get-HeaderConstant {
    param([string]$Name)
    $header = Get-Content -Raw (Join-Path $script:RepoRoot 'relay\include\wpiemu\relay\relay_options.h')
    $m = [regex]::Match($header, "$Name\s*=\s*(\d+)")
    if (-not $m.Success) { throw "constant $Name not found in relay_options.h" }
    [int]$m.Groups[1].Value
}

$azCmd = Get-Command az -ErrorAction SilentlyContinue
if ($null -eq $azCmd) {
    Write-Host 'az is not installed; install the Azure CLI (it provides az bicep).' -ForegroundColor Red
    exit 1
}

$mainFile = Join-Path $script:RelayDir 'main.bicep'
$template = $null

Invoke-Test 'main.bicep compiles with no warnings or errors' {
    $r = Invoke-Az @('bicep', 'build', '--file', $mainFile, '--stdout')
    Assert-True ($r.ExitCode -eq 0) 'exit code 0'
    Assert-True ($r.Diagnostics.Count -eq 0) 'no diagnostics'
    if ($r.Diagnostics.Count -gt 0) { $r.Diagnostics | ForEach-Object { Write-Host "       $_" } }
    $script:template = $r.Stdout | ConvertFrom-Json
    Assert-True ($null -ne $script:template) 'compiled ARM JSON parses'
}

Invoke-Test 'az bicep lint passes' {
    $r = Invoke-Az @('bicep', 'lint', '--file', $mainFile)
    Assert-True ($r.ExitCode -eq 0) 'exit code 0'
    Assert-True ($r.Diagnostics.Count -eq 0) 'no diagnostics'
}

$paramFiles = @{}
foreach ($envName in 'dev', 'prod') {
    Invoke-Test "main.$envName.bicepparam compiles and binds to main.bicep" {
        $r = Invoke-Az @('bicep', 'build-params', '--file', (Join-Path $script:RelayDir "main.$envName.bicepparam"), '--stdout')
        Assert-True ($r.ExitCode -eq 0) 'exit code 0'
        Assert-True ($r.Diagnostics.Count -eq 0) 'no diagnostics'
        if ($r.Diagnostics.Count -gt 0) { $r.Diagnostics | ForEach-Object { Write-Host "       $_" } }
        $parameters = ($r.Stdout | ConvertFrom-Json).parametersJson | ConvertFrom-Json
        $script:paramFiles[$envName] = $parameters.parameters
        $given = @($parameters.parameters.PSObject.Properties.Name)
        $declared = @($script:template.parameters.PSObject.Properties.Name)
        $required = @($script:template.parameters.PSObject.Properties | Where-Object { $_.Value.PSObject.Properties.Name -notcontains 'defaultValue' } | ForEach-Object Name)
        Assert-True (@($given | Where-Object { $_ -notin $declared }).Count -eq 0) 'sets only parameters that main.bicep declares'
        Assert-True (@($required | Where-Object { $_ -notin $given }).Count -eq 0) 'sets every required parameter'
        Assert-Equal $parameters.parameters.environmentName.value $envName 'environmentName matches the file'
        Assert-True ($parameters.parameters.deployRelay.value -eq $false) 'committed file has deployRelay=false'
        Assert-Equal $parameters.parameters.relayImage.value '' 'committed file has an empty relayImage'
        Assert-Equal $parameters.parameters.location.value 'southcentralus' 'location is southcentralus'
        Assert-True ($parameters.parameters.logRetentionDays.value -ge 30) 'log retention is set'
        Assert-True ([double]$parameters.parameters.dailyLogCapGb.value -gt 0) 'daily log cap is set'
    }
}

Invoke-Test 'dev and prod differ in caps and budget' {
    Assert-True ($paramFiles['dev'].monthlyBudgetUsd.value -lt $paramFiles['prod'].monthlyBudgetUsd.value) 'dev budget is lower than prod'
    Assert-True ($paramFiles['dev'].maxRooms.value -lt $paramFiles['prod'].maxRooms.value) 'dev has fewer rooms than prod'
}

Invoke-Test 'template parameters' {
    $p = $template.parameters
    Assert-Equal $p.location.defaultValue 'southcentralus' 'location defaults to southcentralus'
    Assert-Equal @($p.environmentName.allowedValues) @('dev', 'prod') 'environmentName allows dev and prod'
    Assert-Equal $p.udpPort.defaultValue (Get-HeaderConstant 'kDefaultRelayPort') 'udpPort default equals the relay default port'
    Assert-True ($p.maxRooms.maxValue -le (Get-HeaderConstant 'kMaxConfigurableRooms')) 'maxRooms cap does not exceed the relay limit'
    foreach ($name in 'location', 'environmentName', 'namePrefix', 'dnsLabelSuffix', 'relayImage', 'deployRelay', 'udpPort', 'cpu', 'memoryGb',
        'maxRooms', 'roomTtlMs', 'peerBytesPerSec', 'logRetentionDays', 'dailyLogCapGb', 'alertEmail', 'monthlyBudgetUsd', 'tags') {
        Assert-True ($p.PSObject.Properties.Name -contains $name) "parameter $name exists"
    }
    Assert-True ($p.udpPort.metadata.description -match '443') 'udpPort documents the UDP 443 trade-off'
    Assert-Equal $p.relayImage.defaultValue '' 'relayImage may be empty'
}

Invoke-Test 'outputs' {
    $names = @($template.outputs.PSObject.Properties.Name)
    foreach ($n in 'relayFqdn', 'relayIp', 'acrLoginServer', 'acrName', 'identityPrincipalId', 'relayEndpoint') {
        Assert-True ($names -contains $n) "output $n exists"
    }
    $text = $template.outputs | ConvertTo-Json -Depth 20
    Assert-True ($text -notmatch 'listKeys|primarySharedKey|workspaceKey|listCredentials|password') 'no output exposes a key or password'
    $leaks = @(Get-ModuleDeployments $template | ForEach-Object { $_.properties.template.outputs | ConvertTo-Json -Depth 20 } | Where-Object { $_ -match 'listKeys|primarySharedKey|workspaceKey|listCredentials|password' })
    Assert-True ($leaks.Count -eq 0) 'no module output exposes a key or password'
}

Invoke-Test 'registry' {
    $acr = @(Get-ArmResources $template | Where-Object { $_.Resource.type -eq 'Microsoft.ContainerRegistry/registries' -and $_.Resource.PSObject.Properties.Name -contains 'sku' })[0].Resource
    Assert-Equal $acr.sku.name 'Basic' 'ACR SKU is Basic'
    Assert-True ($acr.properties.adminUserEnabled -eq $false) 'adminUserEnabled is false'
    Assert-True ($acr.properties.anonymousPullEnabled -eq $false) 'anonymous pull is off'
    Assert-True ($template.variables.registryName -match 'uniqueString') 'registry name derives from uniqueString'
}

Invoke-Test 'identity and AcrPull' {
    $all = @(Get-ArmResources $template)
    Assert-True (@($all | Where-Object { $_.Resource.type -eq 'Microsoft.ManagedIdentity/userAssignedIdentities' }).Count -eq 1) 'one user-assigned identity'
    $ra = @($all | Where-Object { $_.Resource.type -eq 'Microsoft.Authorization/roleAssignments' })
    Assert-True ($ra.Count -eq 1) 'one role assignment'
    $role = $ra[0].Resource
    $identityModule = @(Get-ModuleDeployments $template | Where-Object { $_.name -eq 'identity' })[0].properties.template
    Assert-True ($identityModule.variables.acrPullRoleDefinitionId -match '7f951dda-4ed3-4680-a7ca-43fe172d538d') 'role definition is AcrPull (7f951dda-4ed3-4680-a7ca-43fe172d538d)'
    Assert-True ($role.properties.roleDefinitionId -match 'acrPullRoleDefinitionId') 'the assignment uses that role definition'
    Assert-True ($role.name -match '^\[guid\(') 'role assignment name is a deterministic guid()'
    Assert-True ($role.scope -match 'Microsoft.ContainerRegistry/registries') 'role assignment is scoped to the registry'
    $dependsOnIdentity = @($role.dependsOn) -join ' '
    Assert-True ($dependsOnIdentity -notmatch 'Microsoft.ContainerInstance|OperationalInsights') 'role assignment does not depend on the relay or logs'
}

Invoke-Test 'logs' {
    $ws = @(Get-ArmResources $template | Where-Object { $_.Resource.type -eq 'Microsoft.OperationalInsights/workspaces' -and $_.Resource.PSObject.Properties.Name -contains 'properties' -and $_.Resource.properties.PSObject.Properties.Name -contains 'sku' })[0].Resource
    Assert-Equal $ws.properties.sku.name 'PerGB2018' 'workspace SKU is PerGB2018'
    Assert-True ($ws.properties.PSObject.Properties.Name -contains 'retentionInDays') 'retention is set'
    Assert-True ($ws.properties.workspaceCapping.PSObject.Properties.Name -contains 'dailyQuotaGb') 'daily cap is set'
}

Invoke-Test 'relay container group' {
    $groups = @(Get-ArmResources $template | Where-Object { $_.Resource.type -eq 'Microsoft.ContainerInstance/containerGroups' })
    Assert-True ($groups.Count -eq 1) 'exactly one container group'
    $g = $groups[0]
    Assert-Equal $g.Module 'relay' 'container group lives in the relay module'
    Assert-Equal $g.ModuleCondition "[parameters('deployRelay')]" 'relay module is gated by deployRelay (false: no container group)'
    $cg = $g.Resource
    Assert-Equal $cg.properties.restartPolicy 'Always' 'restartPolicy is Always'
    Assert-Equal $cg.properties.osType 'Linux' 'osType is Linux'
    Assert-Equal $cg.properties.ipAddress.type 'Public' 'public IP address'
    Assert-True ($cg.properties.ipAddress.PSObject.Properties.Name -contains 'dnsNameLabel') 'dnsNameLabel is set'
    Assert-True (@($cg.properties.containers).Count -eq 1) 'one container'
    $ipPorts = @($cg.properties.ipAddress.ports)
    $containerPorts = @($cg.properties.containers[0].properties.ports)
    Assert-True ($ipPorts.Count -eq 1 -and $ipPorts[0].protocol -eq 'UDP') 'one exposed port, UDP'
    Assert-True ($containerPorts.Count -eq 1 -and $containerPorts[0].protocol -eq 'UDP') 'one container port, UDP'
    Assert-True (($cg | ConvertTo-Json -Depth 30) -notmatch '"TCP"') 'no TCP port anywhere'
    Assert-Equal $cg.identity.type 'UserAssigned' 'user-assigned identity'
    Assert-True ($null -ne $cg.properties.imageRegistryCredentials[0].identity) 'image is pulled with the managed identity'
    Assert-True ($cg.properties.imageRegistryCredentials[0].PSObject.Properties.Name -notcontains 'password') 'no registry password'
    $probe = $cg.properties.containers[0].properties.livenessProbe
    Assert-Equal @($probe.exec.command) @('wpiemu_relay', '--selfcheck') 'liveness probe runs wpiemu_relay --selfcheck'
    Assert-True ($probe.periodSeconds -ge 10 -and $probe.failureThreshold -ge 2) 'probe period and threshold are sensible'
    Assert-True ($null -ne $cg.properties.diagnostics.logAnalytics.workspaceId) 'diagnostics go to Log Analytics'
    Assert-True ($cg.properties.containers[0].properties.resources.requests.PSObject.Properties.Name -contains 'cpu') 'resource requests are set'
}

Invoke-Test 'environment variables match the relay' {
    $relayNames = Get-RelayEnvNames
    Assert-True ($relayNames.Count -ge 10) 'found the relay settings in relay_options.cpp'
    $envNames = @($template.variables.relayEnvironment | ForEach-Object { $_.name })
    Assert-True ($envNames.Count -ge 5) 'the template sets relay variables'
    foreach ($n in $envNames) { Assert-True ($n -in $relayNames) "$n is a real relay setting" }
    foreach ($n in 'WPIEMU_RELAY_PORT', 'WPIEMU_RELAY_MAX_ROOMS', 'WPIEMU_RELAY_ROOM_TTL_MS', 'WPIEMU_RELAY_PEER_BYTES_PER_SEC', 'WPIEMU_RELAY_LOG') {
        Assert-True ($n -in $envNames) "$n is set"
    }
    $log = @($template.variables.relayEnvironment | Where-Object { $_.name -eq 'WPIEMU_RELAY_LOG' })[0]
    Assert-Equal $log.value 'json' 'relay logs JSON'
    $selfcheck = Get-Content -Raw (Join-Path $script:RepoRoot 'relay\src\relay_options.cpp')
    Assert-True ($selfcheck -match '--selfcheck') 'the relay still has --selfcheck'
}

Invoke-Test 'budget' {
    $module = @(Get-ModuleDeployments $template | Where-Object { $_.name -eq 'budget' })[0]
    Assert-Equal $module.condition "[not(empty(parameters('alertEmail')))]" 'budget is skipped without alertEmail'
    $b = @(Get-ArmResources $template | Where-Object { $_.Resource.type -eq 'Microsoft.Consumption/budgets' })[0].Resource
    Assert-Equal $b.properties.timeGrain 'Monthly' 'monthly budget'
    $n = $b.properties.notifications
    Assert-True ($n.Actual_GreaterThan_80_Percent.threshold -eq 80 -and $n.Actual_GreaterThan_80_Percent.thresholdType -eq 'Actual') 'alert at 80% actual'
    Assert-True ($n.Forecasted_GreaterThan_100_Percent.threshold -eq 100 -and $n.Forecasted_GreaterThan_100_Percent.thresholdType -eq 'Forecasted') 'alert at 100% forecast'
}

Invoke-Test 'negative checks: a broken parameter file is rejected' {
    New-Item -ItemType Directory -Force -Path $script:Scratch | Out-Null
    try {
        $missing = Join-Path $script:Scratch 'missing.bicepparam'
        Set-Content -Path $missing -Value "using '../../main.bicep'`nparam environmentName = 'dev'`n"
        $r = Invoke-Az @('bicep', 'build-params', '--file', $missing, '--stdout')
        Assert-True ($r.ExitCode -ne 0 -or $r.Diagnostics.Count -gt 0) 'a missing required parameter (dnsLabelSuffix) is reported'

        $unknown = Join-Path $script:Scratch 'unknown.bicepparam'
        Set-Content -Path $unknown -Value "using '../../main.bicep'`nparam environmentName = 'dev'`nparam dnsLabelSuffix = '-x'`nparam noSuchParameter = 1`n"
        $r = Invoke-Az @('bicep', 'build-params', '--file', $unknown, '--stdout')
        Assert-True ($r.ExitCode -ne 0 -or $r.Diagnostics.Count -gt 0) 'an unknown parameter is reported'

        $badEnv = Join-Path $script:Scratch 'badenv.bicepparam'
        Set-Content -Path $badEnv -Value "using '../../main.bicep'`nparam environmentName = 'staging'`nparam dnsLabelSuffix = '-x'`n"
        $r = Invoke-Az @('bicep', 'build-params', '--file', $badEnv, '--stdout')
        Assert-True ($r.ExitCode -ne 0 -or $r.Diagnostics.Count -gt 0) 'a value outside @allowed is reported'
    }
    finally {
        Remove-Item -Recurse -Force -Path $script:Scratch -ErrorAction SilentlyContinue
    }
}

Write-Host ''
Write-Host "$script:Passed passed, $script:Failures failed"
if ($script:Failures -gt 0) { exit 1 }
exit 0
