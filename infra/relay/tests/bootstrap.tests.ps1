#Requires -Version 7.0
# Plain PowerShell tests for infra/relay/bootstrap.ps1 (no Pester needed). No test talks to Azure or GitHub:
# `az` and `gh` are replaced by an in-memory fake world through the same injection points the script exposes.
# Run: pwsh -NoProfile -File infra/relay/tests/bootstrap.tests.ps1
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot '..\bootstrap.ps1')

$script:Failures = 0
$script:Passed = 0
$script:Scratch = Join-Path $PSScriptRoot '.scratch-bootstrap'
New-Item -ItemType Directory -Force -Path $script:Scratch | Out-Null
$script:ScratchDir = $script:Scratch
$script:Sleep = { param([int]$Seconds) }

function Assert-True {
    param([bool]$Condition, [string]$Name)
    if ($Condition) { $script:Passed++; Write-Host "  ok   $Name" }
    else { $script:Failures++; Write-Host "  FAIL $Name" -ForegroundColor Red }
}

function Assert-Equal {
    param($Actual, $Expected, [string]$Name)
    $same = ($null -eq $Actual -and $null -eq $Expected) -or ($null -ne $Actual -and $null -ne $Expected -and "$Actual" -ceq "$Expected")
    if ($same) { $script:Passed++; Write-Host "  ok   $Name" }
    else { $script:Failures++; Write-Host "  FAIL $Name`n         expected: $Expected`n         actual:   $Actual" -ForegroundColor Red }
}

function Invoke-Test {
    param([string]$Name, [scriptblock]$Body)
    Write-Host $Name
    try { & $Body }
    catch { $script:Failures++; Write-Host "  FAIL threw: $($_.Exception.Message)`n$($_.ScriptStackTrace)" -ForegroundColor Red }
    finally { $script:AzRunner = $null; $script:GhRunner = $null; $script:ConfirmRunner = $null }
}

# ---- constants the tests assert against, written out literally on purpose ---------------------------------------------

$Sub = '3fa85f64-5717-4562-b3fc-2c963f66afa6'
$Tenant = '9a1b2c3d-1111-2222-3333-444455556666'
$HumanOid = 'dddddddd-0000-0000-0000-000000000001'
$DevRg = 'rg-wpiemu-relay-dev'
$ProdRg = 'rg-wpiemu-relay-prod'
$DevScope = "/subscriptions/$Sub/resourceGroups/$DevRg"
$ProdScope = "/subscriptions/$Sub/resourceGroups/$ProdRg"
$ContributorId = 'b24988ac-6180-42a0-ab88-20f7382dd24c'
$ReaderId = 'acdd72a7-3385-48ef-bd42-f606fba81ae7'
$RbacAdminId = 'f58310d9-a9f6-439a-9e8d-f62e7b41a168'
$AcrPullId = '7f951dda-4ed3-4680-a7ca-43fe172d538d'
$ExpectedCondition = "((!(ActionMatches{'Microsoft.Authorization/roleAssignments/write'})) OR (@Request[Microsoft.Authorization/roleAssignments:RoleDefinitionId] ForAnyOfAnyValues:GuidEquals {$AcrPullId})) AND ((!(ActionMatches{'Microsoft.Authorization/roleAssignments/delete'})) OR (@Resource[Microsoft.Authorization/roleAssignments:RoleDefinitionId] ForAnyOfAnyValues:GuidEquals {$AcrPullId}))"
$DevClient = 'bbbbbbbb-0000-0000-0000-000000000001'
$DevSp = 'cccccccc-0000-0000-0000-000000000001'
$ProdClient = 'bbbbbbbb-0000-0000-0000-000000000002'
$ProdSp = 'cccccccc-0000-0000-0000-000000000002'
$PrClient = 'bbbbbbbb-0000-0000-0000-000000000003'
$PrSp = 'cccccccc-0000-0000-0000-000000000003'
$allTypeRegions = @('South Central US', 'Central US', 'East US 2', 'East US', 'West US 2', 'West US 3', 'North Central US')

# ---- fake world -------------------------------------------------------------------------------------------------------

function New-World {
    $script:World = @{
        Groups = @{}; Apps = [System.Collections.Generic.List[object]]::new(); Sps = [System.Collections.Generic.List[object]]::new()
        Creds = @{}; Assignments = [System.Collections.Generic.List[object]]::new(); RoleDefs = [System.Collections.Generic.List[object]]::new()
        Acrs = @{}; AppN = 0; AsgN = 0; DefN = 0
        Calls = [System.Collections.Generic.List[string]]::new(); Raw = [System.Collections.Generic.List[string]]::new()
        FailPatterns = @(); Overrides = [ordered]@{}; HumanRoles = @('Owner'); GraphRoles = @('Application Administrator')
        Gh = @{
            Authed = $true; Envs = @{}; Policies = @{}; Vars = @{ '' = @{}; 'relay-dev' = @{}; 'relay-prod' = @{} }
            Calls = [System.Collections.Generic.List[string]]::new(); Raw = [System.Collections.Generic.List[string]]::new()
        }
    }
    $script:AzRunner = $script:FakeAz
    $script:GhRunner = $script:FakeGh
    $script:World.Assignments.Add(@{ id = 'human-owner'; principalId = $HumanOid; scope = "/subscriptions/$Sub"; roleDefinitionId = '/x/8e3af657-a8ff-443c-a75c-2fe8c4bcb635'; roleDefinitionName = 'Owner'; condition = $null })
}

function New-Ok { param([string]$Text = '') [pscustomobject]@{ ExitCode = 0; Output = $Text; StdOut = $Text } }
function New-Fail { param([string]$Text) [pscustomobject]@{ ExitCode = 1; Output = $Text; StdOut = '' } }
function Get-Opt { param([string[]]$A, [string]$Name) $i = [array]::IndexOf($A, $Name); if ($i -ge 0 -and $i + 1 -lt $A.Count) { $A[$i + 1] } else { $null } }
function ConvertTo-Js { param($Value) ConvertTo-Json -InputObject $Value -Compress -Depth 8 }
function Read-JsonArg { param([string]$Arg) (Get-Content -LiteralPath ($Arg.TrimStart('@')) -Raw) | ConvertFrom-Json -AsHashtable }

$script:RawAzMutation = '^(group (create|update|delete)|ad (app|sp) (create|delete)|ad app federated-credential (create|update|delete)|role (assignment|definition) (create|update|delete)|provider register|bicep install)'
$script:RawGhMutation = '^(api -X (PUT|POST|PATCH|DELETE)|variable (set|delete))'

$script:RoleNames = @{ $ContributorId = 'Contributor'; $ReaderId = 'Reader'; $RbacAdminId = 'Role Based Access Control Administrator'; $AcrPullId = 'AcrPull' }

$script:FakeAz = {
    param([string[]]$AzArgs)
    $w = $script:World
    $cmd = $AzArgs -join ' '
    $w.Calls.Add($cmd)
    if ($cmd -match $script:RawAzMutation) { $w.Raw.Add($cmd) }
    foreach ($k in $w.Overrides.Keys) { if ($cmd -match $k) { return $w.Overrides[$k] } }
    foreach ($f in $w.FailPatterns) { if ($cmd -match $f) { return New-Fail "ERROR: simulated failure for: $cmd" } }

    if ($cmd -match '^account show') {
        return New-Ok (ConvertTo-Js @{ id = $Sub; name = 'Test Sub'; state = 'Enabled'; tenantId = $Tenant; user = @{ name = 'dev@example.org'; type = 'user' } })
    }
    if ($cmd -match '^account list-locations') { return New-Ok '["South Central US"]' }
    if ($cmd -match '^provider show .*registrationState') { return New-Ok '"Registered"' }
    if ($cmd -match '^provider show') { return New-Ok (ConvertTo-Js $allTypeRegions) }
    if ($cmd -match '^bicep version') { return New-Ok 'Bicep CLI version 0.30.3' }
    if ($cmd -match '^rest .*graph\.microsoft\.com.*directoryRole') { return New-Ok (ConvertTo-Js @($w.GraphRoles)) }
    if ($cmd -match '^rest .*graph\.microsoft\.com.*authorizationPolicy') { return New-Ok 'true' }
    if ($cmd -match '^rest ') {
        return New-Ok (@{ value = @(
                    @{ name = @{ value = 'ContainerGroups' }; currentValue = 3; limit = 100 },
                    @{ name = @{ value = 'StandardCores' }; currentValue = 4; limit = 100 }) } | ConvertTo-Json -Depth 6 -Compress)
    }
    if ($cmd -match '^ad signed-in-user show') { return New-Ok "`"$HumanOid`"" }

    if ($cmd -match '^group exists') { return New-Ok $(if ($w.Groups.ContainsKey((Get-Opt $AzArgs '--name'))) { 'true' } else { 'false' }) }
    if ($cmd -match '^group show') {
        $g = $w.Groups[(Get-Opt $AzArgs '--name')]
        return New-Ok (ConvertTo-Js @{ name = (Get-Opt $AzArgs '--name'); location = $g.location; tags = $g.tags })
    }
    if ($cmd -match '^group create') {
        $tags = @{}
        foreach ($t in ($AzArgs | Where-Object { $_ -match '^[a-zA-Z]+=' -and $_ -notmatch '^-' })) { $kv = $t -split '=', 2; $tags[$kv[0]] = $kv[1] }
        $w.Groups[(Get-Opt $AzArgs '--name')] = @{ location = (Get-Opt $AzArgs '--location'); tags = $tags }
        return New-Ok
    }
    if ($cmd -match '^group update') {
        $g = $w.Groups[(Get-Opt $AzArgs '--name')]
        foreach ($t in ($AzArgs | Where-Object { $_ -match '^tags\.' })) { $kv = ($t -replace '^tags\.', '') -split '=', 2; $g.tags[$kv[0]] = $kv[1] }
        return New-Ok
    }
    if ($cmd -match '^group delete') { $w.Groups.Remove((Get-Opt $AzArgs '--name')); return New-Ok }

    if ($cmd -match '^ad app list') {
        $name = Get-Opt $AzArgs '--display-name'
        return New-Ok (ConvertTo-Js @($w.Apps | Where-Object { $_.displayName -like "$name*" }))
    }
    if ($cmd -match '^ad app create') {
        $w.AppN++
        $app = @{ id = ('aaaaaaaa-0000-0000-0000-{0:D12}' -f $w.AppN); appId = ('bbbbbbbb-0000-0000-0000-{0:D12}' -f $w.AppN); displayName = (Get-Opt $AzArgs '--display-name'); passwordCredentials = @() }
        $w.Apps.Add($app); $w.Creds[$app.appId] = [System.Collections.Generic.List[object]]::new()
        return New-Ok (ConvertTo-Js @{ id = $app.id; appId = $app.appId })
    }
    if ($cmd -match '^ad app delete') {
        $id = Get-Opt $AzArgs '--id'
        $null = $w.Apps.RemoveAll([Predicate[object]] { param($a) $a.appId -eq $id })
        $null = $w.Sps.RemoveAll([Predicate[object]] { param($s) $s.appId -eq $id })
        return New-Ok
    }
    if ($cmd -match '^ad sp list') {
        $appId = [regex]::Match((Get-Opt $AzArgs '--filter'), "'([^']+)'").Groups[1].Value
        return New-Ok (ConvertTo-Js @($w.Sps | Where-Object { $_.appId -eq $appId }))
    }
    if ($cmd -match '^ad sp create') {
        $appId = Get-Opt $AzArgs '--id'
        $n = [int]($appId.Substring($appId.Length - 4))
        $sp = @{ id = ('cccccccc-0000-0000-0000-{0:D12}' -f $n); appId = $appId }
        $w.Sps.Add($sp)
        return New-Ok (ConvertTo-Js $sp)
    }
    if ($cmd -match '^ad app federated-credential list') { return New-Ok (ConvertTo-Js @($w.Creds[(Get-Opt $AzArgs '--id')])) }
    if ($cmd -match '^ad app federated-credential create') {
        $body = Read-JsonArg (Get-Opt $AzArgs '--parameters')
        $w.Creds[(Get-Opt $AzArgs '--id')].Add($body)
        return New-Ok
    }
    if ($cmd -match '^ad app federated-credential update') {
        $body = Read-JsonArg (Get-Opt $AzArgs '--parameters')
        $list = $w.Creds[(Get-Opt $AzArgs '--id')]
        $idx = 0; for ($i = 0; $i -lt $list.Count; $i++) { if ($list[$i].name -eq (Get-Opt $AzArgs '--federated-credential-id')) { $idx = $i } }
        $list[$idx] = $body
        return New-Ok
    }
    if ($cmd -match '^ad app federated-credential delete') {
        $name = Get-Opt $AzArgs '--federated-credential-id'
        $null = $w.Creds[(Get-Opt $AzArgs '--id')].RemoveAll([Predicate[object]] { param($c) $c.name -eq $name })
        return New-Ok
    }

    if ($cmd -match '^role assignment list') {
        $who = Get-Opt $AzArgs '--assignee'
        return New-Ok (ConvertTo-Js @($w.Assignments | Where-Object { $_.principalId -eq $who }))
    }
    if ($cmd -match '^role assignment create') {
        $w.AsgN++
        $role = Get-Opt $AzArgs '--role'
        $defId = $null; $defName = $null
        if ($script:RoleNames.ContainsKey($role)) { $defId = "/subscriptions/$Sub/providers/Microsoft.Authorization/roleDefinitions/$role"; $defName = $script:RoleNames[$role] }
        else {
            $def = $w.RoleDefs | Where-Object { $_.roleName -eq $role } | Select-Object -First 1
            if ($null -eq $def) { return New-Fail "ERROR: role '$role' not found" }
            $defId = $def.id; $defName = $def.roleName
        }
        $w.Assignments.Add(@{
                id = "$(Get-Opt $AzArgs '--scope')/providers/Microsoft.Authorization/roleAssignments/asg-$($w.AsgN)"
                principalId = (Get-Opt $AzArgs '--assignee-object-id'); scope = (Get-Opt $AzArgs '--scope')
                roleDefinitionId = $defId; roleDefinitionName = $defName
                condition = (Get-Opt $AzArgs '--condition'); conditionVersion = (Get-Opt $AzArgs '--condition-version')
            })
        return New-Ok
    }
    if ($cmd -match '^role assignment delete') {
        $id = Get-Opt $AzArgs '--ids'
        $null = $w.Assignments.RemoveAll([Predicate[object]] { param($a) $a.id -eq $id })
        return New-Ok
    }
    if ($cmd -match '^role definition list') {
        $name = Get-Opt $AzArgs '--name'
        return New-Ok (ConvertTo-Js @($w.RoleDefs | Where-Object { $_.roleName -eq $name }))
    }
    if ($cmd -match '^role definition (create|update)') {
        $body = Read-JsonArg (Get-Opt $AzArgs '--role-definition')
        $existing = $w.RoleDefs | Where-Object { $_.roleName -eq $body.Name } | Select-Object -First 1
        $id = if ($existing) { $existing.id } else { $w.DefN++; "/subscriptions/$Sub/providers/Microsoft.Authorization/roleDefinitions/eeeeeeee-0000-0000-0000-$('{0:D12}' -f $w.DefN)" }
        if ($existing) { $null = $w.RoleDefs.Remove($existing) }
        $w.RoleDefs.Add(@{ id = $id; roleName = $body.Name; permissions = @(@{ actions = @($body.Actions) }); assignableScopes = @($body.AssignableScopes); jsonBody = $body })
        return New-Ok
    }
    if ($cmd -match '^role definition delete') {
        $name = Get-Opt $AzArgs '--name'
        $null = $w.RoleDefs.RemoveAll([Predicate[object]] { param($d) $d.roleName -eq $name })
        return New-Ok
    }
    if ($cmd -match '^acr list') {
        $rg = Get-Opt $AzArgs '--resource-group'
        if (-not $w.Groups.ContainsKey($rg)) { return New-Fail "ERROR: (ResourceGroupNotFound) Resource group '$rg' could not be found." }
        return New-Ok (ConvertTo-Js @($w.Acrs[$rg]))
    }
    New-Fail "unexpected az call: $cmd"
}

$script:FakeGh = {
    param([string[]]$GhArgs)
    $g = $script:World.Gh
    $cmd = $GhArgs -join ' '
    $g.Calls.Add($cmd)
    if ($cmd -match $script:RawGhMutation) { $g.Raw.Add($cmd) }
    if ($cmd -match '^auth status') { if ($g.Authed) { return New-Ok 'Logged in' } else { return New-Fail 'You are not logged into any GitHub hosts. To log in, run: gh auth login' } }
    if ($cmd -match '^api users/(\S+) --jq') { return New-Ok '4242' }
    if ($cmd -match '^api -X PUT repos/[^/]+/[^/]+/environments/(\S+)') {
        $name = $Matches[1]
        $body = if ($GhArgs -contains '--input') { Read-JsonArg (Get-Opt $GhArgs '--input') } else { $null }
        $rules = @()
        $policy = @{ protected_branches = $false; custom_branch_policies = $false }
        if ($body) {
            $rules = @(@{ type = 'required_reviewers'; reviewers = @($body.reviewers | ForEach-Object { @{ type = $_.type; reviewer = @{ id = $_.id } } }) })
            $policy = $body.deployment_branch_policy
        }
        $g.Envs[$name] = @{ name = $name; protection_rules = $rules; deployment_branch_policy = $policy }
        return New-Ok
    }
    if ($cmd -match '^api -X POST repos/[^/]+/[^/]+/environments/(\S+)/deployment-branch-policies') {
        $name = $Matches[1]
        if (-not $g.Policies.ContainsKey($name)) { $g.Policies[$name] = @() }
        $g.Policies[$name] += @{ name = ((Get-Opt $GhArgs '-f') -replace '^name=', '') }
        return New-Ok
    }
    if ($cmd -match '^api repos/[^/]+/[^/]+/environments/(\S+)/deployment-branch-policies') {
        $list = if ($g.Policies.ContainsKey($Matches[1])) { @($g.Policies[$Matches[1]]) } else { @() }
        return New-Ok (ConvertTo-Js @{ branch_policies = $list })
    }
    if ($cmd -match '^api repos/[^/]+/[^/]+/environments/(\S+)$') {
        if ($g.Envs.ContainsKey($Matches[1])) { return New-Ok (ConvertTo-Js $g.Envs[$Matches[1]]) }
        return New-Fail 'gh: Not Found (HTTP 404)'
    }
    if ($cmd -match '^variable list') {
        $envName = Get-Opt $GhArgs '--env'; if (-not $envName) { $envName = '' }
        $map = $g.Vars[$envName]
        return New-Ok (ConvertTo-Js @($map.Keys | ForEach-Object { @{ name = $_; value = $map[$_] } }))
    }
    if ($cmd -match '^variable set') {
        $envName = Get-Opt $GhArgs '--env'; if (-not $envName) { $envName = '' }
        $g.Vars[$envName][$GhArgs[2]] = (Get-Opt $GhArgs '--body')
        return New-Ok
    }
    New-Fail "unexpected gh call: $cmd"
}

# ---- run helpers ------------------------------------------------------------------------------------------------------

# Runs the bootstrap and returns the run object plus the captured log text, the final report and the raw JSON text.
function Invoke-Run {
    param([hashtable]$Options = @{}, [switch]$KeepLog)
    $opts = @{ Force = $true; AlertEmail = 'ops@example.org' }
    foreach ($k in $Options.Keys) { $opts[$k] = $Options[$k] }
    $out = @(& { Invoke-Bootstrap -Options $opts } 6>&1)
    $run = $out | Where-Object { $_ -isnot [System.Management.Automation.InformationRecord] } | Select-Object -Last 1
    $log = ($out | Where-Object { $_ -is [System.Management.Automation.InformationRecord] } | ForEach-Object { [string]$_.MessageData }) -join "`n"
    $report = Get-FinalReport
    [pscustomobject]@{ Run = $run; Log = $log; Report = $report; Executed = @($run.Executed | ForEach-Object { ($_.Args -join ' ') }); ExitCode = $run.ExitCode }
}

function Get-Status { param($R, [string]$StepName, [string]$Like = '*') @($R.Run.Items | Where-Object { $_.Step -eq $StepName -and $_.Item -like $Like } | ForEach-Object { $_.Status }) }
function Get-AllStatuses { param($R, [string[]]$Steps) @($R.Run.Items | Where-Object { $Steps -contains $_.Step } | ForEach-Object { $_.Status }) | Sort-Object -Unique }

function Get-ExpectedSetupSequence {
    $sub = ''
    @(
        "group create --name $DevRg --location southcentralus --tags app=wpiemu-relay environment=dev managedBy=bootstrap -o none",
        "group create --name $ProdRg --location southcentralus --tags app=wpiemu-relay environment=prod managedBy=bootstrap -o none",
        'ad app create --display-name wpiemu-relay-deploy-dev --sign-in-audience AzureADMyOrg -o json',
        "ad sp create --id $DevClient -o json",
        'ad app create --display-name wpiemu-relay-deploy-prod --sign-in-audience AzureADMyOrg -o json',
        "ad sp create --id $ProdClient -o json",
        'ad app create --display-name wpiemu-relay-whatif --sign-in-audience AzureADMyOrg -o json',
        "ad sp create --id $PrClient -o json",
        "ad app federated-credential create --id $DevClient --parameters @{{JSONFILE}}",
        "ad app federated-credential create --id $ProdClient --parameters @{{JSONFILE}}",
        "ad app federated-credential create --id $PrClient --parameters @{{JSONFILE}}",
        "role assignment create --assignee-object-id $DevSp --assignee-principal-type ServicePrincipal --role $ContributorId --scope $DevScope -o none",
        "role assignment create --assignee-object-id $DevSp --assignee-principal-type ServicePrincipal --role $RbacAdminId --scope $DevScope --condition $ExpectedCondition --condition-version 2.0 -o none",
        "role assignment create --assignee-object-id $ProdSp --assignee-principal-type ServicePrincipal --role $ContributorId --scope $ProdScope -o none",
        "role assignment create --assignee-object-id $ProdSp --assignee-principal-type ServicePrincipal --role $RbacAdminId --scope $ProdScope --condition $ExpectedCondition --condition-version 2.0 -o none",
        'role definition create --role-definition @{{JSONFILE}} -o none',
        "role assignment create --assignee-object-id $PrSp --assignee-principal-type ServicePrincipal --role $ReaderId --scope $DevScope -o none",
        "role assignment create --assignee-object-id $PrSp --assignee-principal-type ServicePrincipal --role wpiemu-relay-whatif --scope $DevScope -o none"
    )
}

# ---- tests ------------------------------------------------------------------------------------------------------------

Invoke-Test 'ABAC condition string is exact (AcrPull only, write via Request, delete via Resource)' {
    $c = New-RoleAssignmentCondition -RoleIds @($AcrPullId)
    Assert-Equal $c $ExpectedCondition 'exact condition text'
    $two = New-RoleAssignmentCondition -RoleIds @('AAAAAAAA-0000-0000-0000-000000000001', '11111111-0000-0000-0000-000000000002')
    Assert-True ($two.Contains('GuidEquals {aaaaaaaa-0000-0000-0000-000000000001, 11111111-0000-0000-0000-000000000002}')) 'multiple ids are comma separated and lower-cased'
    Assert-True (Test-ConditionEquivalent $c ($c -replace ' ', '  ')) 'whitespace differences are equivalent'
    Assert-True (Test-ConditionEquivalent $c $c.ToUpperInvariant()) 'case differences are equivalent'
    Assert-True (-not (Test-ConditionEquivalent $c $two)) 'different role sets differ'
    Assert-True (-not (Test-ConditionEquivalent $c '')) 'empty differs'
    Assert-True (Test-ConditionEquivalent '' $null) 'empty equals null'
}

Invoke-Test 'DNS suffix derivation is deterministic, 6 lowercase hex chars' {
    $a = Get-DefaultDnsSuffix $Sub
    Assert-Equal $a (Get-DefaultDnsSuffix $Sub.ToUpperInvariant()) 'case-insensitive and stable'
    Assert-True ($a -match '^[0-9a-f]{6}$') 'six lowercase hex chars'
    Assert-True ($a -ne (Get-DefaultDnsSuffix '00000000-0000-0000-0000-000000000000')) 'differs per subscription'
    $expected = [System.Convert]::ToHexString([System.Security.Cryptography.SHA256]::HashData([System.Text.Encoding]::UTF8.GetBytes($Sub))).ToLowerInvariant().Substring(0, 6)
    Assert-Equal $a $expected 'first 6 hex chars of SHA-256 of the lower-case id'
}

Invoke-Test 'dry run makes zero mutating calls and prints every command it would run' {
    New-World
    $r = Invoke-Run @{}
    Assert-Equal $r.ExitCode 0 'exit code 0'
    Assert-Equal $script:World.Raw.Count 0 'zero mutating az calls reached az'
    Assert-Equal $script:World.Gh.Raw.Count 0 'zero mutating gh calls'
    Assert-Equal $r.Run.Executed.Count 0 'wrapper executed no mutating call'
    Assert-Equal $r.Run.Planned.Count (Get-ExpectedSetupSequence).Count 'all setup commands planned'
    Assert-True (@($r.Report.PlannedCommands | Where-Object { $_ -match 'role assignment create .*<dev-sp-object-id>' }).Count -eq 2) 'placeholders stand in for ids that do not exist yet (dev app)'
    Assert-True (@($r.Report.PlannedCommands | Where-Object { $_ -match 'role assignment create .*<prod-sp-object-id>' }).Count -eq 2) 'placeholders stand in for ids that do not exist yet (prod app)'
    Assert-Equal $script:World.Groups.Count 0 'no resource group exists afterwards'
    Assert-Equal $script:World.Apps.Count 0 'no app exists afterwards'
    Assert-True ($r.Log -match 'dry run') 'announces dry run'
    Assert-True (@(Get-Status $r 'ResourceGroups') -contains 'WouldCreate') 'reports WouldCreate'
}

Invoke-Test '-Apply on an empty subscription issues the exact sequence' {
    New-World
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.ExitCode 0 'exit code 0'
    $expected = Get-ExpectedSetupSequence
    Assert-Equal $r.Executed.Count $expected.Count 'number of mutating calls'
    for ($i = 0; $i -lt $expected.Count; $i++) { Assert-Equal $r.Executed[$i] $expected[$i] "call $($i + 1): $($expected[$i].Substring(0, [Math]::Min(70, $expected[$i].Length)))" }
    Assert-Equal @($script:World.Raw).Count $expected.Count 'raw az saw the same number of mutations'

    $creds = @($r.Run.Executed | Where-Object { $_.Json })
    $cred = @($creds | Where-Object { $_.Args -contains 'federated-credential' -or $_.Args[2] -eq 'federated-credential' } | ForEach-Object { $_.Json | ConvertFrom-Json })
    Assert-Equal $cred.Count 3 'three federated credentials'
    Assert-Equal $cred[0].subject 'repo:Overture-7421/wpi-emulator:environment:relay-dev' 'dev environment subject'
    Assert-Equal $cred[1].subject 'repo:Overture-7421/wpi-emulator:environment:relay-prod' 'prod environment subject'
    Assert-Equal $cred[2].subject 'repo:Overture-7421/wpi-emulator:pull_request' 'pull_request subject'
    Assert-True (@($cred | Where-Object { $_.issuer -ne 'https://token.actions.githubusercontent.com' }).Count -eq 0) 'issuer on every credential'
    Assert-True (@($cred | Where-Object { @($_.audiences) -notcontains 'api://AzureADTokenExchange' }).Count -eq 0) 'audience on every credential'
    Assert-Equal @($script:World.Creds[$DevClient]).Count 1 'dev app holds exactly one credential'
    Assert-Equal $script:World.Creds[$DevClient][0].subject 'repo:Overture-7421/wpi-emulator:environment:relay-dev' 'dev app holds the relay-dev subject only'
    Assert-Equal @($script:World.Creds[$ProdClient]).Count 1 'prod app holds exactly one credential'
    Assert-Equal $script:World.Creds[$ProdClient][0].subject 'repo:Overture-7421/wpi-emulator:environment:relay-prod' 'prod app holds the relay-prod subject only'
    Assert-Equal @($script:World.Creds[$PrClient]).Count 1 'what-if app holds only the pull_request credential'

    $roleCalls = @($r.Run.Executed | Where-Object { $_.Args[0] -eq 'role' -and $_.Args[2] -eq 'create' -and $_.Args[1] -eq 'assignment' })
    Assert-True (@($roleCalls | Where-Object { $_.Args -contains 'Owner' -or ($_.Args -join ' ') -match '8e3af657-a8ff-443c-a75c-2fe8c4bcb635|18d7d88d-d35e-4fb5-a5c3-7773c20a72d9' }).Count -eq 0) 'Owner and User Access Administrator are never assigned'
    $scopes = @($roleCalls | ForEach-Object { Get-Opt $_.Args '--scope' })
    Assert-True (@($scopes | Where-Object { $_ -notmatch '/resourceGroups/rg-wpiemu-relay-(dev|prod)$' }).Count -eq 0) 'every scope is a managed resource group, never the subscription'
    $prCalls = @($roleCalls | Where-Object { (Get-Opt $_.Args '--assignee-object-id') -eq $PrSp })
    Assert-Equal @($prCalls | ForEach-Object { Get-Opt $_.Args '--role' }) @($ReaderId, 'wpiemu-relay-whatif') 'pull-request identity gets only Reader plus the what-if role'
    Assert-True (@($prCalls | Where-Object { (Get-Opt $_.Args '--scope') -ne $DevScope }).Count -eq 0) 'pull-request identity is limited to the dev resource group'
    $def = $script:World.RoleDefs[0].jsonBody
    Assert-Equal ($def.Actions -join ',') 'Microsoft.Resources/deployments/whatIf/action,Microsoft.Resources/deployments/validate/action,Microsoft.Resources/deployments/read,Microsoft.Resources/deployments/operations/read' 'what-if role actions'
    Assert-Equal ($def.AssignableScopes -join ',') $DevScope 'what-if role is assignable only at the dev resource group'
    Assert-True (@($script:World.Groups[$DevRg].tags.Keys | Sort-Object) -join ',' -eq 'app,environment,managedBy') 'resource group tags'
    Assert-Equal $script:World.Groups[$ProdRg].tags['environment'] 'prod' 'prod environment tag'

    $statuses = Get-AllStatuses $r @('ResourceGroups', 'Identity', 'FederatedCredentials', 'RoleAssignments')
    Assert-Equal ($statuses -join ',') 'Created' 'every item reports Created'
    Assert-True (@(Get-Status $r 'GrantPromotion') -contains 'Skipped') 'GrantPromotion skipped (no dev registry yet)'
}

Invoke-Test 'second run with everything present: zero mutating calls, everything Exists' {
    New-World
    $null = Invoke-Run @{ Apply = $true }
    $script:World.Raw.Clear()
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.ExitCode 0 'exit code 0'
    Assert-Equal $r.Run.Executed.Count 0 'no mutating call executed'
    Assert-Equal $script:World.Raw.Count 0 'none reached az'
    Assert-Equal ((Get-AllStatuses $r @('ResourceGroups', 'Identity', 'FederatedCredentials', 'RoleAssignments')) -join ',') 'Exists' 'every item reports Exists'
    Assert-Equal $r.Run.Warnings.Count 0 'no warnings'
}

Invoke-Test 'partial state: only the missing credential is created' {
    New-World
    $null = Invoke-Run @{ Apply = $true }
    $null = $script:World.Creds[$ProdClient].RemoveAll([Predicate[object]] { param($c) $c.name -eq 'github-relay-prod' })
    $script:World.Raw.Clear()
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.Executed.Count 1 'exactly one mutation'
    Assert-Equal $r.Executed[0] "ad app federated-credential create --id $ProdClient --parameters @{{JSONFILE}}" 'it is the credential create on the prod app'
    Assert-Equal (($r.Run.Executed[0].Json | ConvertFrom-Json).subject) 'repo:Overture-7421/wpi-emulator:environment:relay-prod' 'for the prod subject'
    Assert-Equal (@(Get-Status $r 'FederatedCredentials' '*relay-prod') -join ',') 'Created' 'reported Created'
    Assert-Equal (@(Get-Status $r 'FederatedCredentials' '*relay-dev') -join ',') 'Exists' 'the other stays Exists'
}

Invoke-Test 'app exists without a service principal or credentials: reused, not duplicated' {
    New-World
    $script:World.AppN = 1
    $app = @{ id = 'aaaaaaaa-0000-0000-0000-000000000001'; appId = $DevClient; displayName = 'wpiemu-relay-deploy-dev'; passwordCredentials = @() }
    $script:World.Apps.Add($app); $script:World.Creds[$DevClient] = [System.Collections.Generic.List[object]]::new()
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.ExitCode 0 'exit code 0'
    Assert-Equal @($script:World.Apps | Where-Object { $_.displayName -eq 'wpiemu-relay-deploy-dev' }).Count 1 'still one dev app'
    Assert-True (@($r.Executed | Where-Object { $_ -match 'ad app create --display-name wpiemu-relay-deploy-dev' }).Count -eq 0) 'no second app create for the dev app'
    Assert-True (@($r.Executed | Where-Object { $_ -eq "ad sp create --id $DevClient -o json" }).Count -eq 1) 'service principal created'
    Assert-Equal (@(Get-Status $r 'Identity' 'app wpiemu-relay-deploy-dev') -join ',') 'Exists' 'app reported Exists'
}

Invoke-Test 'wrong credential subject is updated (by name), the right ones are untouched' {
    New-World
    $null = Invoke-Run @{ Apply = $true }
    $script:World.Creds[$ProdClient][0].subject = 'repo:Overture-7421/wpi-emulator:environment:wrong'
    $script:World.Raw.Clear()
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.Executed.Count 1 'exactly one mutation'
    Assert-Equal $r.Executed[0] "ad app federated-credential update --id $ProdClient --federated-credential-id github-relay-prod --parameters @{{JSONFILE}}" 'update by credential name'
    Assert-Equal (($r.Run.Executed[0].Json | ConvertFrom-Json).subject) 'repo:Overture-7421/wpi-emulator:environment:relay-prod' 'new subject'
    Assert-Equal (@(Get-Status $r 'FederatedCredentials' '*relay-prod') -join ',') 'Updated' 'reported Updated'
    Assert-Equal $script:World.Creds[$ProdClient][0].subject 'repo:Overture-7421/wpi-emulator:environment:relay-prod' 'world holds the corrected subject'
}

Invoke-Test 'a credential with the right subject under another name is accepted as is' {
    New-World
    $null = Invoke-Run @{ Apply = $true }
    $script:World.Creds[$DevClient][0].name = 'hand-made'
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.Executed.Count 0 'no mutation'
}

Invoke-Test 'role assignment with a different condition is replaced (Updated)' {
    New-World
    $null = Invoke-Run @{ Apply = $true }
    $bad = $script:World.Assignments | Where-Object { $_.condition -and $_.scope -eq $DevScope } | Select-Object -First 1
    $bad.condition = New-RoleAssignmentCondition -RoleIds @($ContributorId)
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.Executed.Count 2 'delete plus create'
    Assert-True ($r.Executed[0] -match '^role assignment delete --ids ') 'old assignment removed'
    Assert-True ($r.Executed[1] -match "--role $RbacAdminId --scope $([regex]::Escape($DevScope)) --condition ") 'conditioned assignment recreated'
}

Invoke-Test 'a reformatted but equivalent condition from Azure is not rewritten' {
    New-World
    $null = Invoke-Run @{ Apply = $true }
    foreach ($a in $script:World.Assignments) { if ($a.condition) { $a.condition = ($a.condition -replace '\(\(', '( (' -replace '\)\)', ') )').ToUpperInvariant() } }
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.Executed.Count 0 'idempotent'
}

Invoke-Test 'unexpected privileged assignments are reported, never silently kept quiet' {
    New-World
    $null = Invoke-Run @{ Apply = $true }
    $script:World.Assignments.Add(@{ id = "$DevScope/providers/Microsoft.Authorization/roleAssignments/x"; principalId = $DevSp; scope = $DevScope; roleDefinitionId = "/x/8e3af657-a8ff-443c-a75c-2fe8c4bcb635"; roleDefinitionName = 'Owner'; condition = $null })
    $script:World.Assignments.Add(@{ id = "/subscriptions/$Sub/providers/Microsoft.Authorization/roleAssignments/y"; principalId = $DevSp; scope = "/subscriptions/$Sub"; roleDefinitionId = "/x/$ReaderId"; roleDefinitionName = 'Reader'; condition = $null })
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.ExitCode 0 'reported as warnings, not failures'
    Assert-Equal $r.Executed.Count 0 'it does not remove what it did not create'
    Assert-True (@($r.Run.Warnings | Where-Object { $_ -match "unexpected role 'Owner'" }).Count -eq 1) 'warns about Owner in the app''s own resource group'
    Assert-True (@($r.Run.Warnings | Where-Object { $_ -match 'subscription-scope' }).Count -eq 1) 'warns about subscription scope'
}

Invoke-Test 'a deploy app holding a role on the other environment is a failure (exit 1), never auto-removed' {
    New-World
    $null = Invoke-Run @{ Apply = $true }
    $script:World.Assignments.Add(@{ id = "$ProdScope/providers/Microsoft.Authorization/roleAssignments/x"; principalId = $DevSp; scope = $ProdScope; roleDefinitionId = "/x/$ContributorId"; roleDefinitionName = 'Contributor'; condition = $null })
    $script:World.Assignments.Add(@{ id = "$DevScope/providers/Microsoft.Authorization/roleAssignments/z"; principalId = $ProdSp; scope = $DevScope; roleDefinitionId = "/x/$ContributorId"; roleDefinitionName = 'Contributor'; condition = $null })
    $script:World.Raw.Clear()
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.ExitCode 1 'exit code 1'
    Assert-Equal @($r.Run.Items | Where-Object { $_.Status -eq 'Failed' -and $_.Item -match 'cross-environment' }).Count 2 'both cross-environment grants reported'
    Assert-Equal $r.Executed.Count 0 'nothing is changed automatically'
}

Invoke-Test 'prod app: the AcrPull grant on the dev registry is allowed, any other dev-side role is not' {
    New-World
    $null = Invoke-Run @{ Apply = $true }
    $reg = "$DevScope/providers/Microsoft.ContainerRegistry/registries/r1"
    $script:World.Assignments.Add(@{ id = "$reg/providers/Microsoft.Authorization/roleAssignments/p"; principalId = $ProdSp; scope = $reg; roleDefinitionId = "/x/$AcrPullId"; roleDefinitionName = 'AcrPull'; condition = $null })
    Assert-Equal (Invoke-Run @{ Apply = $true }).ExitCode 0 'AcrPull on the dev registry is tolerated for the prod app'
    $script:World.Assignments.Add(@{ id = "$reg/providers/Microsoft.Authorization/roleAssignments/q"; principalId = $ProdSp; scope = $reg; roleDefinitionId = "/x/$ContributorId"; roleDefinitionName = 'Contributor'; condition = $null })
    Assert-Equal (Invoke-Run @{ Apply = $true }).ExitCode 1 'Contributor on the dev registry is not'
    New-World
    $null = Invoke-Run @{ Apply = $true }
    $script:World.Assignments.Add(@{ id = "$reg/providers/Microsoft.Authorization/roleAssignments/p"; principalId = $DevSp; scope = $reg; roleDefinitionId = "/x/$AcrPullId"; roleDefinitionName = 'AcrPull'; condition = $null })
    Assert-Equal (Invoke-Run @{ Apply = $true }).ExitCode 0 'the dev app having AcrPull on its own registry is just its own scope'
}

Invoke-Test 'the assignment guard refuses cross-environment scopes outright' {
    New-World
    $registryOfProd = "$ProdScope/providers/Microsoft.ContainerRegistry/registries/prodreg"
    $registryOfDev = "$DevScope/providers/Microsoft.ContainerRegistry/registries/devreg"
    foreach ($case in @(
            @{ Key = 'dev'; Scope = $ProdScope; Role = $ContributorId; Name = 'dev app, prod RG' },
            @{ Key = 'dev'; Scope = $registryOfProd; Role = $AcrPullId; Name = 'dev app, prod registry' },
            @{ Key = 'dev'; Scope = "/subscriptions/$Sub"; Role = $ContributorId; Name = 'dev app, subscription' },
            @{ Key = 'prod'; Scope = $DevScope; Role = $ContributorId; Name = 'prod app, dev RG' },
            @{ Key = 'prod'; Scope = $registryOfDev; Role = $ContributorId; Name = 'prod app, dev registry with another role' },
            @{ Key = 'prod'; Scope = $registryOfDev; Role = $RbacAdminId; Name = 'prod app, dev registry RBAC admin' },
            @{ Key = 'pr'; Scope = $ProdScope; Role = $ReaderId; Name = 'what-if app, prod RG' })) {
        $threw = $false
        try { Assert-AssignmentAllowed -AppKey $case.Key -Scope $case.Scope -RoleId $case.Role } catch { $threw = $true }
        Assert-True $threw "refused: $($case.Name)"
    }
    foreach ($case in @(
            @{ Key = 'dev'; Scope = $DevScope; Role = $ContributorId; Name = 'dev app, dev RG' },
            @{ Key = 'dev'; Scope = $registryOfDev; Role = $AcrPullId; Name = 'dev app, own registry' },
            @{ Key = 'prod'; Scope = $ProdScope; Role = $RbacAdminId; Name = 'prod app, prod RG' },
            @{ Key = 'prod'; Scope = $registryOfDev; Role = $AcrPullId; Name = 'prod app, AcrPull on dev registry (promotion)' },
            @{ Key = 'pr'; Scope = $DevScope; Role = $ReaderId; Name = 'what-if app, dev RG' })) {
        $threw = $false
        try { Assert-AssignmentAllowed -AppKey $case.Key -Scope $case.Scope -RoleId $case.Role } catch { $threw = $true }
        Assert-True (-not $threw) "allowed: $($case.Name)"
    }
}

Invoke-Test 'GrantPromotion: skipped while the dev registry is missing, granted once it exists, then idempotent' {
    New-World
    $null = Invoke-Run @{ Apply = $true }
    $script:World.Raw.Clear()
    $r = Invoke-Run @{ Apply = $true; Step = @('GrantPromotion') }
    Assert-Equal $r.ExitCode 0 'exit code 0 when skipped'
    Assert-Equal $r.Executed.Count 0 'nothing mutated'
    $item = @($r.Run.Items | Where-Object { $_.Step -eq 'GrantPromotion' })[0]
    Assert-Equal $item.Status 'Skipped' 'Skipped'
    Assert-True ($item.Detail -match 'dev registry does not exist yet' -and $item.Detail -match '-Step GrantPromotion -Apply') 'clear message with the re-run command'

    Assert-Equal @($script:World.Assignments | Where-Object { $_.principalId -eq $ProdSp -and $_.scope -like "$DevScope*" }).Count 0 'before GrantPromotion the prod app has nothing on the dev side'
    $registryId = "$DevScope/providers/Microsoft.ContainerRegistry/registries/wpiemureldev1234"
    $script:World.Acrs[$DevRg] = @(@{ id = $registryId; name = 'wpiemureldev1234' })
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.ExitCode 0 'a full run with the registry present'
    Assert-Equal $r.Executed.Count 1 'a full run grants promotion too (GrantPromotion is part of All)'
    $script:World.Assignments.RemoveAll([Predicate[object]] { param($a) $a.scope -eq $registryId }) | Out-Null
    $r = Invoke-Run @{ Apply = $true; Step = @('GrantPromotion') }
    Assert-Equal $r.Executed.Count 1 'one grant'
    Assert-Equal $r.Executed[0] "role assignment create --assignee-object-id $ProdSp --assignee-principal-type ServicePrincipal --role $AcrPullId --scope $registryId -o none" 'AcrPull for the PROD app on the dev registry, no condition'
    Assert-Equal (@(Get-Status $r 'GrantPromotion') -join ',') 'Created' 'Created'
    $prodAssignments = @($script:World.Assignments | Where-Object { $_.principalId -eq $ProdSp })
    $onDevSide = @($prodAssignments | Where-Object { $_.scope -like "$DevScope*" })
    Assert-Equal $onDevSide.Count 1 'the prod app holds exactly one role on the dev side after GrantPromotion'
    Assert-Equal $onDevSide[0].scope $registryId 'and it is on the dev registry itself'
    Assert-Equal $onDevSide[0].roleDefinitionName 'AcrPull' 'and it is AcrPull'
    Assert-Equal @($script:World.Assignments | Where-Object { $_.principalId -eq $DevSp -and $_.scope -notlike "$DevScope*" }).Count 0 'the dev app received nothing outside the dev RG'
    $r = Invoke-Run @{ Apply = $true; Step = @('GrantPromotion') }
    Assert-Equal $r.Executed.Count 0 'rerun is a no-op'
    Assert-Equal (@(Get-Status $r 'GrantPromotion') -join ',') 'Exists' 'Exists'
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.Run.Warnings.Count 0 'the promotion grant is not flagged as unexpected in a full run'
    Assert-Equal $r.Executed.Count 0 'full run stays idempotent with the grant present'
}

Invoke-Test 'GrantPromotion dry run with the registry present only plans the grant' {
    New-World
    $null = Invoke-Run @{ Apply = $true }
    $script:World.Acrs[$DevRg] = @(@{ id = "$DevScope/providers/Microsoft.ContainerRegistry/registries/r1"; name = 'r1' })
    $script:World.Raw.Clear()
    $r = Invoke-Run @{ Step = @('GrantPromotion') }
    Assert-Equal $script:World.Raw.Count 0 'no mutation reached az'
    Assert-Equal $r.Run.Planned.Count 1 'one planned command'
    Assert-Equal (@(Get-Status $r 'GrantPromotion') -join ',') 'WouldCreate' 'WouldCreate'
}

Invoke-Test '-Step limits the work and reads the existing app without creating it' {
    New-World
    $null = Invoke-Run @{ Apply = $true }
    $script:World.Raw.Clear()
    $r = Invoke-Run @{ Apply = $true; Step = @('RoleAssignments') }
    Assert-Equal $r.Executed.Count 0 'role step alone is idempotent'
    Assert-True (@($r.Run.Items | Where-Object { $_.Step -ne 'RoleAssignments' }).Count -eq 0) 'only the selected step ran'
    New-World
    $r = Invoke-Run @{ Apply = $true; Step = @('RoleAssignments') }
    Assert-Equal $r.ExitCode 1 'role step without the app exits 1'
    Assert-True (@($r.Run.Items | Where-Object { $_.Status -eq 'Failed' -and $_.Detail -match 'run the Identity step first' }).Count -ge 1) 'tells to run Identity first'
    Assert-Equal $r.Executed.Count 0 'nothing mutated'
}

Invoke-Test 'skipped pull-request identity, and the old shared mode no longer exists' {
    New-World
    $r = Invoke-Run @{ Apply = $true; SkipPrIdentity = $true }
    Assert-Equal $r.ExitCode 0 'exit code 0'
    Assert-Equal $script:World.Apps.Count 2 'skip: only the two deploy apps'
    Assert-Equal @($script:World.Creds[$DevClient]).Count 1 'skip: dev app has one credential'
    Assert-Equal @($script:World.Creds[$ProdClient]).Count 1 'skip: prod app has one credential'
    Assert-Equal $script:World.RoleDefs.Count 0 'skip: no custom role'
    Assert-True (-not $r.Report.GitHub.Variables.Repository.Contains('AZURE_PR_CLIENT_ID')) 'skip: no PR variables'
    New-World
    Assert-Equal (Invoke-Run @{ SharedPrIdentity = $true }).ExitCode 2 '-SharedPrIdentity is rejected (a pull request must never get deploy rights)'
    Assert-Equal (Invoke-Run @{ AppName = 'x' }).ExitCode 2 'the old single-app -AppName is rejected'
}

Invoke-Test 'custom app name prefix and per-app names' {
    New-World
    $r = Invoke-Run @{ Apply = $true; AppNamePrefix = 'acme-deploy'; PrAppName = 'acme-whatif' }
    Assert-Equal $r.ExitCode 0 'exit code 0'
    Assert-Equal (($script:World.Apps | ForEach-Object { $_.displayName }) -join ',') 'acme-deploy-dev,acme-deploy-prod,acme-whatif' 'display names'
    Assert-Equal (Invoke-Run @{ AppNamePrefix = 'x'; PrAppName = 'x-dev' }).ExitCode 2 'the what-if name may not collide with a deploy name'
}

Invoke-Test 'no role assignment ever crosses environments (dev app: dev RG only, prod app: prod RG only)' {
    New-World
    $null = Invoke-Run @{ Apply = $true }
    $script:World.Acrs[$DevRg] = @(@{ id = "$DevScope/providers/Microsoft.ContainerRegistry/registries/r1"; name = 'r1' })
    $null = Invoke-Run @{ Apply = $true }
    $dev = @($script:World.Assignments | Where-Object { $_.principalId -eq $DevSp })
    $prod = @($script:World.Assignments | Where-Object { $_.principalId -eq $ProdSp })
    $pr = @($script:World.Assignments | Where-Object { $_.principalId -eq $PrSp })
    Assert-Equal $dev.Count 2 'dev app: Contributor + conditioned RBAC admin, nothing else'
    Assert-True (@($dev | Where-Object { $_.scope -ne $DevScope }).Count -eq 0) '(a) every dev app assignment is on the dev RG'
    Assert-True (@($dev | Where-Object { $_.scope -like "$ProdScope*" }).Count -eq 0) '(d) the dev app has nothing under the prod RG, registry of prod included'
    Assert-Equal $prod.Count 3 'prod app: Contributor + conditioned RBAC admin + the promotion grant'
    Assert-True (@($prod | Where-Object { $_.scope -like "$ProdScope*" }).Count -eq 2) '(a) the prod app has its two roles on the prod RG'
    Assert-True (@($prod | Where-Object { $_.scope -notlike "$ProdScope*" -and -not ($_.roleDefinitionName -eq 'AcrPull' -and $_.scope -like "$DevScope/providers/Microsoft.ContainerRegistry/registries/*") }).Count -eq 0) '(a) the prod app holds nothing on the dev side except AcrPull on the dev registry'
    Assert-True (@($pr | Where-Object { $_.scope -ne $DevScope }).Count -eq 0) 'what-if app: dev RG only'
    Assert-Equal @($script:World.Creds[$DevClient]).Count 1 '(b) dev app has exactly one credential'
    Assert-Equal @($script:World.Creds[$ProdClient]).Count 1 '(b) prod app has exactly one credential'
    Assert-True (@($script:World.Creds[$DevClient] | Where-Object { $_.subject -notmatch ':environment:relay-dev$' }).Count -eq 0) '(b) the dev credential is the relay-dev environment'
    Assert-True (@($script:World.Creds[$ProdClient] | Where-Object { $_.subject -notmatch ':environment:relay-prod$' }).Count -eq 0) '(b) the prod credential is the relay-prod environment'
    Assert-True (@(@($script:World.Creds[$DevClient]) + @($script:World.Creds[$PrClient]) | Where-Object { $_.subject -match 'relay-prod' }).Count -eq 0) 'no relay-prod subject on the dev or what-if app'
    Assert-True (@($script:World.Creds[$ProdClient] | Where-Object { $_.subject -match 'relay-dev|pull_request' }).Count -eq 0) 'no relay-dev or pull_request subject on the prod app'
}

Invoke-Test 'every planned and executed role command of the dev app targets the dev RG, of the prod app the prod RG (or the dev registry as AcrPull)' {
    New-World
    $script:World.Acrs[$DevRg] = @()
    $dry = Invoke-Run @{}
    $app = Invoke-Run @{ Apply = $true }
    $script:World.Acrs[$DevRg] = @(@{ id = "$DevScope/providers/Microsoft.ContainerRegistry/registries/r1"; name = 'r1' })
    $promo = Invoke-Run @{ Apply = $true; Step = @('GrantPromotion') }
    $calls = @($app.Run.Executed) + @($promo.Run.Executed) + @($dry.Run.Planned)
    $devCalls = @($calls | Where-Object { $_.Args[0] -eq 'role' -and $_.Args -contains '--assignee-object-id' -and (((Get-Opt $_.Args '--assignee-object-id') -eq $DevSp) -or (Get-Opt $_.Args '--assignee-object-id') -eq '<dev-sp-object-id>') })
    $prodCalls = @($calls | Where-Object { $_.Args[0] -eq 'role' -and $_.Args -contains '--assignee-object-id' -and (((Get-Opt $_.Args '--assignee-object-id') -eq $ProdSp) -or (Get-Opt $_.Args '--assignee-object-id') -eq '<prod-sp-object-id>') })
    Assert-True ($devCalls.Count -ge 4) 'dev role commands were seen'
    Assert-True ($prodCalls.Count -ge 5) 'prod role commands were seen'
    Assert-True (@($devCalls | Where-Object { (Get-Opt $_.Args '--scope') -ne $DevScope }).Count -eq 0) 'dev app commands: dev RG scope only'
    Assert-True (@($prodCalls | Where-Object { (Get-Opt $_.Args '--scope') -ne $ProdScope -and -not ((Get-Opt $_.Args '--role') -eq $AcrPullId -and (Get-Opt $_.Args '--scope') -like "$DevScope/providers/Microsoft.ContainerRegistry/registries/*") }).Count -eq 0) 'prod app commands: prod RG scope, plus AcrPull on the dev registry'
    Assert-Equal @($app.Run.Executed | Where-Object { $_.Args -contains $AcrPullId }).Count 0 'before GrantPromotion nothing grants AcrPull to anyone'
    Assert-Equal @($promo.Run.Executed | Where-Object { $_.Args -contains $AcrPullId }).Count 1 'GrantPromotion grants AcrPull exactly once'
}

Invoke-Test '-Remove dry run changes nothing; -Remove -Apply tears down but keeps resource groups' {
    New-World
    $null = Invoke-Run @{ Apply = $true }
    $script:World.Raw.Clear()
    $r = Invoke-Run @{ Remove = $true }
    Assert-Equal $script:World.Raw.Count 0 'dry-run remove: zero mutations'
    Assert-Equal $script:World.Apps.Count 3 'apps still present'
    Assert-True (@(Get-Status $r 'RoleAssignments') -contains 'WouldRemove') 'WouldRemove reported'
    Assert-Equal $r.Run.Planned.Count 13 'plans: 6 assignment deletes, the custom role, 3 credentials, 3 apps'

    $r = Invoke-Run @{ Remove = $true; Apply = $true }
    Assert-Equal $r.ExitCode 0 'exit code 0'
    Assert-Equal $script:World.Apps.Count 0 'apps removed'
    Assert-Equal $script:World.Assignments.Count 1 'only the signed-in human assignment is left'
    Assert-Equal $script:World.RoleDefs.Count 0 'custom role removed'
    Assert-Equal $script:World.Groups.Count 2 'resource groups kept'
    Assert-True ($r.Executed[0] -match '^role assignment delete') 'assignments removed first'
    Assert-True ($r.Executed[-1] -match '^ad app delete') 'app registrations removed last'
    Assert-True ((@($r.Executed | Where-Object { $_ -match '^group delete' })).Count -eq 0) 'no resource group delete'
    $r2 = Invoke-Run @{ Remove = $true; Apply = $true }
    Assert-Equal $r2.Executed.Count 0 'removing again is a no-op'
    Assert-True ((Get-AllStatuses $r2 @('RoleAssignments', 'FederatedCredentials', 'Identity')) -contains 'Absent') 'reports Absent'
}

Invoke-Test '-Remove -RemoveResourceGroups deletes only groups tagged managedBy=bootstrap' {
    New-World
    $null = Invoke-Run @{ Apply = $true }
    $script:World.Groups[$ProdRg].tags.Remove('managedBy')
    $r = Invoke-Run @{ Remove = $true; Apply = $true; RemoveResourceGroups = $true }
    Assert-Equal $r.ExitCode 0 'exit code 0'
    Assert-True (-not $script:World.Groups.ContainsKey($DevRg)) 'tagged dev group deleted'
    Assert-True ($script:World.Groups.ContainsKey($ProdRg)) 'untagged prod group kept'
    Assert-Equal (@(Get-Status $r 'ResourceGroups' "$ProdRg*") -join ',') 'Skipped' 'prod reported Skipped with the reason'
    $r = Invoke-Run @{ RemoveResourceGroups = $true }
    Assert-Equal $r.ExitCode 2 '-RemoveResourceGroups without -Remove is a usage error'
}

Invoke-Test 'GitHub variables and commands are printed exactly' {
    New-World
    $r = Invoke-Run @{ Apply = $true; ProdReviewers = @('octocat') }
    $gh = $r.Report.GitHub
    Assert-Equal $gh.Repository 'Overture-7421/wpi-emulator' 'repository'
    $suffix = Get-DefaultDnsSuffix $Sub
    $dev = $gh.Variables.Environments['relay-dev']
    Assert-Equal (($dev.Keys) -join ',') 'AZURE_CLIENT_ID,AZURE_TENANT_ID,AZURE_SUBSCRIPTION_ID,RELAY_RESOURCE_GROUP,RELAY_DNS_SUFFIX,RELAY_ALERT_EMAIL' 'six per-environment variables'
    Assert-Equal $dev['AZURE_CLIENT_ID'] $DevClient 'relay-dev AZURE_CLIENT_ID is the dev app'
    Assert-Equal $gh.Variables.Environments['relay-prod']['AZURE_CLIENT_ID'] $ProdClient 'relay-prod AZURE_CLIENT_ID is the prod app'
    Assert-Equal $dev['RELAY_RESOURCE_GROUP'] $DevRg 'dev resource group'
    Assert-Equal $gh.Variables.Environments['relay-prod']['RELAY_RESOURCE_GROUP'] $ProdRg 'prod resource group'
    Assert-Equal $dev['RELAY_DNS_SUFFIX'] $suffix 'dns suffix from the subscription'
    Assert-Equal $gh.Variables.Repository['AZURE_LOCATION'] 'southcentralus' 'repository AZURE_LOCATION'
    Assert-Equal $gh.Variables.Repository['AZURE_PR_CLIENT_ID'] $PrClient 'repository AZURE_PR_CLIENT_ID is the what-if app'
    $cmds = $gh.CommandsFull
    Assert-True ($cmds -contains 'gh api -X PUT repos/Overture-7421/wpi-emulator/environments/relay-dev') 'dev environment command'
    Assert-True (@($cmds | Where-Object { $_ -match "^'\{.*\}' \| gh api -X PUT repos/Overture-7421/wpi-emulator/environments/relay-prod --input -$" }).Count -eq 1) 'prod environment command pipes a JSON body'
    Assert-True (@($cmds | Where-Object { $_ -match 'custom_branch_policies":true' }).Count -eq 1) 'prod body restricts to custom branch policies'
    Assert-True (@($cmds | Where-Object { $_ -match '"reviewers":\[\{"type":"User","id":"<id of octocat>"\}\]' }).Count -eq 1) 'reviewer placeholder until resolved'
    Assert-True ($cmds -contains 'gh api -X POST repos/Overture-7421/wpi-emulator/environments/relay-prod/deployment-branch-policies -f name=main -f type=branch') 'main-only branch policy command'
    Assert-True ($cmds -contains "gh variable set AZURE_CLIENT_ID --env relay-dev --repo Overture-7421/wpi-emulator --body $DevClient") 'variable set with --env (dev app)'
    Assert-True ($cmds -contains "gh variable set AZURE_CLIENT_ID --env relay-prod --repo Overture-7421/wpi-emulator --body $ProdClient") 'variable set with --env (prod app)'
    Assert-True (@($cmds | Where-Object { $_ -match "AZURE_CLIENT_ID --env relay-dev .*$ProdClient|AZURE_CLIENT_ID --env relay-prod .*$DevClient" }).Count -eq 0) 'client ids are never swapped between environments'
    Assert-Equal @($r.Report.Apps | ForEach-Object { $_.DisplayName }) @('wpiemu-relay-deploy-dev', 'wpiemu-relay-deploy-prod', 'wpiemu-relay-whatif') 'report lists the three apps'
    Assert-Equal @($r.Report.Apps | ForEach-Object { $_.ClientId }) @($DevClient, $ProdClient, $PrClient) 'with their client ids'
    Assert-Equal $r.Report.Apps[0].RoleScope $DevScope 'dev app role scope'
    Assert-Equal $r.Report.Apps[1].RoleScope $ProdScope 'prod app role scope'
    Assert-True ($cmds -contains 'gh variable set AZURE_LOCATION --repo Overture-7421/wpi-emulator --body southcentralus') 'repository variable set'
    Assert-True (@($cmds | Where-Object { $_ -match '^gh variable set .* --env' }).Count -eq 12) 'twelve environment variables'
    Assert-True (@($cmds | Where-Object { $_ -match 'secret' }).Count -eq 0) 'no secret commands'
    Assert-Equal $script:World.Gh.Raw.Count 0 'printing mutates nothing on GitHub'
    Assert-True (@(Get-Status $r 'GitHub') -contains 'Skipped') 'GitHub step reports Skipped (print only)'
}

Invoke-Test 'explicit -DnsSuffix and -Repository/-Environment parameters flow into the output' {
    New-World
    $r = Invoke-Run @{ DnsSuffix = 'abc12'; Repository = 'acme/robot'; DevEnvironment = 'd'; ProdEnvironment = 'p'; Apply = $true }
    $gh = $r.Report.GitHub
    Assert-Equal $gh.Variables.Environments['d']['RELAY_DNS_SUFFIX'] 'abc12' 'suffix'
    Assert-True ($gh.CommandsFull -contains 'gh api -X PUT repos/acme/robot/environments/d') 'repo and env names'
    $subjects = @($script:World.Creds[$DevClient] | ForEach-Object { $_.subject }) + @($script:World.Creds[$ProdClient] | ForEach-Object { $_.subject })
    Assert-Equal ($subjects -join ',') 'repo:acme/robot:environment:d,repo:acme/robot:environment:p' 'subjects follow the parameters'
    Assert-Equal ($subjects -join ',') 'repo:acme/robot:environment:d,repo:acme/robot:environment:p' 'subjects follow the parameters'
}

Invoke-Test '-ConfigureGitHub refuses when gh is not authenticated (exit 2, nothing changed)' {
    New-World
    $script:World.Gh.Authed = $false
    $r = Invoke-Run @{ Apply = $true; ConfigureGitHub = $true; ProdReviewers = @('octocat') }
    Assert-Equal $r.ExitCode 2 'exit code 2'
    Assert-Equal $script:World.Raw.Count 0 'no Azure mutation'
    Assert-Equal $script:World.Gh.Raw.Count 0 'no GitHub mutation'
    Assert-True ($r.Log -match 'gh is not authenticated') 'explains why'
    Assert-Equal $script:World.Groups.Count 0 'no resource group was created'
}

Invoke-Test '-ConfigureGitHub needs reviewers and an alert email' {
    New-World
    $r = Invoke-Run @{ Apply = $true; ConfigureGitHub = $true }
    Assert-Equal $r.ExitCode 2 'exit code 2 without reviewers'
    Assert-Equal $script:World.Raw.Count 0 'nothing mutated'
    New-World
    $r = Invoke-Run @{ Apply = $true; ConfigureGitHub = $true; ProdReviewers = @('octocat'); AlertEmail = '' }
    Assert-Equal $r.ExitCode 2 'exit code 2 without alert email'
}

Invoke-Test '-ConfigureGitHub -Apply creates environments, branch policy and variables, then is idempotent' {
    New-World
    $r = Invoke-Run @{ Apply = $true; ConfigureGitHub = $true; ProdReviewers = @('octocat') }
    Assert-Equal $r.ExitCode 0 'exit code 0'
    $g = $script:World.Gh
    Assert-True ($g.Envs.ContainsKey('relay-dev') -and $g.Envs.ContainsKey('relay-prod')) 'both environments exist'
    Assert-Equal @($g.Envs['relay-dev'].protection_rules).Count 0 'dev has no approval rule'
    $prodRule = @($g.Envs['relay-prod'].protection_rules)[0]
    Assert-Equal @($prodRule.reviewers)[0].reviewer.id 4242 'prod reviewer id was resolved through gh api users/octocat'
    Assert-True ($g.Envs['relay-prod'].deployment_branch_policy.custom_branch_policies -eq $true) 'prod restricted to custom branch policies'
    Assert-Equal @($g.Policies['relay-prod'])[0].name 'main' 'only main may deploy to prod'
    Assert-Equal $g.Vars['relay-dev']['AZURE_CLIENT_ID'] $DevClient 'dev AZURE_CLIENT_ID is the dev app'
    Assert-Equal $g.Vars['relay-prod']['AZURE_CLIENT_ID'] $ProdClient 'prod AZURE_CLIENT_ID is the prod app'
    Assert-Equal $g.Vars['relay-prod']['RELAY_RESOURCE_GROUP'] $ProdRg 'prod RELAY_RESOURCE_GROUP'
    Assert-Equal $g.Vars['']['AZURE_LOCATION'] 'southcentralus' 'repo AZURE_LOCATION'
    Assert-Equal $g.Vars['relay-dev'].Count 6 'six dev variables'
    $g.Raw.Clear()
    $r = Invoke-Run @{ Apply = $true; ConfigureGitHub = $true; ProdReviewers = @('octocat') }
    Assert-Equal $g.Raw.Count 0 'rerun issues no gh mutation'
    Assert-Equal ((Get-AllStatuses $r @('GitHub')) -join ',') 'Exists' 'everything Exists'
    $g.Vars['relay-dev']['RELAY_DNS_SUFFIX'] = 'stale'
    $r = Invoke-Run @{ Apply = $true; ConfigureGitHub = $true; ProdReviewers = @('octocat') }
    Assert-Equal $g.Raw.Count 1 'a changed variable is set again'
    Assert-Equal (@(Get-Status $r 'GitHub' 'relay-dev: RELAY_DNS_SUFFIX') -join ',') 'Updated' 'reported Updated'
}

Invoke-Test '-ConfigureGitHub dry run runs no gh mutation' {
    New-World
    $r = Invoke-Run @{ ConfigureGitHub = $true; ProdReviewers = @('octocat') }
    Assert-Equal $r.ExitCode 0 'exit code 0'
    Assert-Equal $script:World.Gh.Raw.Count 0 'no gh mutation'
    Assert-Equal $script:World.Raw.Count 0 'no az mutation'
    Assert-True ($r.Run.Planned.Count -gt 20) 'plans the environment and variable commands as well'
}

Invoke-Test 'a failing step exits 1, stops its dependents and keeps independent steps' {
    New-World
    $script:World.FailPatterns = @('^ad app create --display-name wpiemu-relay-deploy-dev')
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.ExitCode 1 'exit code 1'
    Assert-Equal (@(Get-Status $r 'ResourceGroups') -join ',') 'Created,Created' 'independent step still ran'
    Assert-Equal (@(Get-Status $r 'Identity') -join ',') 'Failed' 'Identity failed (stops at the first failing app)'
    foreach ($s in 'FederatedCredentials', 'RoleAssignments', 'GrantPromotion', 'GitHub') {
        $item = @($r.Run.Items | Where-Object { $_.Step -eq $s })[0]
        Assert-True ($item.Status -eq 'Skipped' -and $item.Detail -match 'depends on failed step') "$s skipped because of the failure"
    }
    Assert-True (@($r.Executed | Where-Object { $_ -match 'federated-credential|role ' }).Count -eq 0) 'no dependent mutation was attempted'
}

Invoke-Test 'a failing role assignment is reported with the az error and exits 1' {
    New-World
    $script:World.FailPatterns = @('^role assignment create .*Role Based|^role assignment create .*f58310d9')
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.ExitCode 1 'exit code 1'
    Assert-True (@($r.Run.Items | Where-Object { $_.Status -eq 'Failed' -and $_.Detail -match 'simulated failure' }).Count -eq 1) 'failure carries the az message'
    Assert-Equal (@(Get-Status $r 'FederatedCredentials' | Sort-Object -Unique) -join ',') 'Created' 'earlier steps completed'
}

Invoke-Test 'directory propagation errors are retried' {
    New-World
    $script:World.Attempts = 0
    $inner = $script:AzRunner
    $script:AzRunner = {
        param([string[]]$AzArgs)
        if (($AzArgs -join ' ') -match '^ad sp create' -and $script:World.Attempts -lt 2) {
            $script:World.Attempts++
            return New-Fail 'ERROR: Resource does not exist or one of its queried reference-property objects are not present.'
        }
        & $inner $AzArgs
    }
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.ExitCode 0 'eventually succeeds'
    Assert-True ($script:World.Attempts -eq 2) 'retried twice'
}

Invoke-Test 'safety: reads cannot smuggle mutations; apply needs confirmation' {
    New-World
    $threw = $false
    try { $null = Read-Az @('group', 'create', '--name', 'x') } catch { $threw = $true }
    Assert-True $threw 'a mutating az command refused as a read'
    $threw = $false
    try { $null = Read-Gh @('variable', 'set', 'X', '--body', 'y') } catch { $threw = $true }
    Assert-True $threw 'a mutating gh command refused as a read'
    $threw = $false
    try { $null = Read-Az @('rest', '--method', 'POST', '--url', 'https://graph.microsoft.com/v1.0/applications') } catch { $threw = $true }
    Assert-True $threw 'az rest POST refused as a read'
    $threw = $false
    try { $null = Read-Gh @('api', 'repos/o/r/environments/e', '-f', 'a=b') } catch { $threw = $true }
    Assert-True $threw 'gh api with fields refused as a read'

    $script:ConfirmRunner = { param($p) $false }
    $r = Invoke-Run @{ Apply = $true; Force = $false }
    Assert-Equal $r.ExitCode 2 'declined confirmation exits 2'
    Assert-Equal $script:World.Raw.Count 0 'nothing mutated after declining'
    Assert-True ($r.Log -notmatch [regex]::Escape($Sub)) 'prompt and log never show the full subscription id'

    $script:ConfirmRunner = { param($p) $true }
    $r = Invoke-Run @{ Apply = $true; Force = $false }
    Assert-Equal $r.ExitCode 0 'accepted confirmation proceeds'
    Assert-True ($script:World.Raw.Count -gt 0) 'mutations happened after confirming'
}

Invoke-Test 'preconditions: not logged in, preflight failure, insufficient permissions' {
    New-World
    $script:World.Overrides['^account show'] = New-Fail "ERROR: Please run 'az login' to setup account."
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.ExitCode 2 'not logged in: exit 2'
    Assert-Equal $script:World.Raw.Count 0 'nothing mutated'

    New-World
    $script:World.Overrides['namespace Microsoft.Consumption .*registrationState'] = New-Ok '"NotRegistered"'
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.ExitCode 2 'preflight failure: exit 2'
    Assert-True (@($r.Run.Preconditions | Where-Object { $_.Id -eq 'preflight' -and $_.Status -eq 'FAIL' }).Count -eq 1) 'preflight reported'
    Assert-Equal $script:World.Raw.Count 0 'nothing mutated'
    $r = Invoke-Run @{ Apply = $true; SkipPreflight = $true }
    Assert-Equal $r.ExitCode 0 '-SkipPreflight proceeds'

    New-World
    $script:World.Assignments.Add(@{ id = 'h'; principalId = $HumanOid; scope = "/subscriptions/$Sub"; roleDefinitionId = '/x/y'; roleDefinitionName = 'Reader'; condition = $null })
    $script:World.Overrides['^role assignment list --assignee dddddddd'] = New-Ok (ConvertTo-Js @(@{ scope = "/subscriptions/$Sub"; roleDefinitionName = 'Reader' }))
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.ExitCode 2 'no Owner/UAA: exit 2'
    Assert-True (@($r.Run.Preconditions | Where-Object { $_.Id -eq 'azure-role' -and $_.Status -eq 'FAIL' }).Count -eq 1) 'azure-role FAIL'
    $r = Invoke-Run @{ Apply = $true; SkipPermissionCheck = $true }
    Assert-Equal $r.ExitCode 0 '-SkipPermissionCheck proceeds'

    New-World
    $script:World.Overrides['^role assignment list --assignee dddddddd'] = New-Ok (ConvertTo-Js @(@{ scope = "/subscriptions/$Sub"; roleDefinitionName = 'Owner' }))
    $script:World.Overrides['^rest .*directoryRole'] = New-Fail 'ERROR: Insufficient privileges'
    $script:World.Overrides['^rest .*authorizationPolicy'] = New-Fail 'ERROR: Forbidden'
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.ExitCode 0 'undeterminable Entra permission is only a warning'
    Assert-True (@($r.Run.Preconditions | Where-Object { $_.Id -eq 'entra' -and $_.Status -eq 'WARN' -and $_.Message -match 'Verify manually' }).Count -eq 1) 'says what to verify manually'

    New-World
    $script:World.Overrides['^role assignment list --assignee dddddddd'] = New-Ok (ConvertTo-Js @(@{ scope = "/subscriptions/$Sub"; roleDefinitionName = 'Owner' }))
    $script:World.GraphRoles = @('Directory Readers')
    $script:World.Overrides['^rest .*authorizationPolicy'] = New-Ok 'false'
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.ExitCode 2 'tenant forbids app registration and no admin role: exit 2'
}

Invoke-Test 'usage errors exit 2' {
    New-World
    Assert-Equal (Invoke-Run @{ SharedPrIdentity = $true; SkipPrIdentity = $true }).ExitCode 2 'removed -SharedPrIdentity'
    Assert-Equal (Invoke-Run @{ DelegatedRoleIds = @('not-a-guid') }).ExitCode 2 'bad role id'
    Assert-Equal (Invoke-Run @{ DevResourceGroup = 'same'; ProdResourceGroup = 'same' }).ExitCode 2 'same resource group'
    Assert-Equal (Invoke-Run @{ Remove = $true; ConfigureGitHub = $true }).ExitCode 2 'remove + configure'
    Assert-Equal $script:World.Calls.Count 0 'usage errors never reach az'
}

Invoke-Test 'ambiguous app registrations are refused, existing client secrets are flagged' {
    New-World
    foreach ($n in 1, 2) { $script:World.Apps.Add(@{ id = "aaaaaaaa-0000-0000-0000-00000000010$n"; appId = "bbbbbbbb-0000-0000-0000-00000000010$n"; displayName = 'wpiemu-relay-deploy-prod'; passwordCredentials = @() }) }
    $r = Invoke-Run @{ Apply = $true }
    Assert-Equal $r.ExitCode 2 'two apps with the name: exit 2'
    Assert-True (@($r.Run.Preconditions | Where-Object { $_.Message -match 'refusing to guess' }).Count -eq 1) 'explains'
    Assert-Equal $script:World.Raw.Count 0 'nothing mutated'

    New-World
    $script:World.Apps.Add(@{ id = 'aaaaaaaa-0000-0000-0000-000000000001'; appId = $DevClient; displayName = 'wpiemu-relay-deploy-dev'; passwordCredentials = @(@{ keyId = 'k' }) })
    $script:World.AppN = 1; $script:World.Creds[$DevClient] = [System.Collections.Generic.List[object]]::new()
    $r = Invoke-Run @{ Apply = $true }
    Assert-True (@($r.Run.Warnings | Where-Object { $_ -match 'client secret' }).Count -eq 1) 'client secret warning'
    Assert-True (@($r.Executed | Where-Object { $_ -match 'credential (reset|delete)' }).Count -eq 0) 'the script never touches secrets itself'
}

Invoke-Test 'no secrets anywhere and ids are masked in the log' {
    New-World
    $r = Invoke-Run @{ Apply = $true; ConfigureGitHub = $true; ProdReviewers = @('octocat') }
    $all = ($r.Log + "`n" + ($r.Report | ConvertTo-Json -Depth 8)) + "`n" + ($r.Executed -join "`n")
    Assert-True ($all -notmatch '(?i)password|client[-_ ]?secret|--secret|create-for-rbac|credential reset|\bsecrets?\b.*=') 'no secret material or secret-creating commands'
    Assert-True ($r.Log -notmatch [regex]::Escape($Sub)) 'full subscription id absent from the log'
    Assert-True ($r.Log -notmatch [regex]::Escape($Tenant)) 'full tenant id absent from the log'
    Assert-True ($r.Log -match '3fa85f64\.\.\.afa6') 'masked subscription id is present'
    Assert-Equal $r.Report.SubscriptionId $Sub 'the report keeps full ids for -Json'
    $scriptText = Get-Content -LiteralPath (Join-Path $PSScriptRoot '..\bootstrap.ps1') -Raw
    Assert-True ($scriptText -notmatch "'(ad','app','credential'|--password|create-for-rbac)") 'script contains no secret-creating command'
}

Invoke-Test 'the JSON report round-trips' {
    New-World
    $r = Invoke-Run @{ ProdReviewers = @('octocat') }
    $j = $r.Report | ConvertTo-Json -Depth 8 | ConvertFrom-Json
    Assert-Equal $j.Mode 'DryRun' 'mode'
    Assert-Equal $j.GitHub.Variables.Environments.'relay-prod'.AZURE_SUBSCRIPTION_ID $Sub 'full subscription id in JSON'
    Assert-True ($j.PlannedCommands.Count -eq 18) 'planned commands listed'
}

Invoke-Test 'script entry point: exit codes and -Json (child process, no az on PATH)' {
    $script = Join-Path $PSScriptRoot '..\bootstrap.ps1'
    $out = & pwsh -NoProfile -File $script -Json -SkipPreflight -RemoveResourceGroups 2>&1
    Assert-Equal $LASTEXITCODE 2 'usage error exits 2 through the real entry point'
    $help = Get-Help $script -Full | Out-String
    Assert-True ($help -match 'Application Administrator' -and $help -match 'ABAC' -and $help -match '-Apply') 'comment-based help documents prerequisites, the ABAC condition and -Apply'
}

Remove-Item -LiteralPath $script:Scratch -Recurse -Force -ErrorAction SilentlyContinue
Write-Host ''
Write-Host "$script:Passed assertions passed, $script:Failures failed"
if ($script:Failures -gt 0) { exit 1 }
exit 0
