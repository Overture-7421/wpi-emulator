<#
.SYNOPSIS
One-time Azure, Entra and GitHub bootstrap for the WPI Emulator relay pipelines (dry run unless -Apply).

.DESCRIPTION
Prepares an Azure subscription and a GitHub repository so the relay workflows can deploy with GitHub OIDC
(no client secrets, ever). Run it ONCE per subscription and repository by a human who owns both. It is
idempotent: re-running with -Apply changes nothing when everything already exists.

SAFE BY DEFAULT. Without -Apply the script is a dry run: it performs read-only queries (az ... show/list,
gh ... list) and prints every mutating command it WOULD run. Every mutating call goes through one wrapper
(Invoke-Tool -Mutating) that is skipped unless -Apply is given. -Apply asks for confirmation unless -Force.

Steps (-Step, default All, run in this order; a failed step makes its dependents Skipped and the exit code 1):
  ResourceGroups        rg-wpiemu-relay-dev and rg-wpiemu-relay-prod (tags app=wpiemu-relay, environment=dev|prod,
                        managedBy=bootstrap).
  Identity              THREE Entra app registrations + service principals, each looked up by exact display name, reused
                        when found, refused when the name is ambiguous:
                          wpiemu-relay-deploy-dev   deploys the dev environment only
                          wpiemu-relay-deploy-prod  deploys the prod environment only
                          wpiemu-relay-whatif       pull-request what-if (omitted with -SkipPrIdentity)
  FederatedCredentials  GitHub OIDC credentials (issuer https://token.actions.githubusercontent.com, audience
                        api://AzureADTokenExchange), created when missing, updated when issuer/subject differ.
                        Each app holds exactly one:
                          deploy-dev:  repo:<owner>/<repo>:environment:relay-dev
                          deploy-prod: repo:<owner>/<repo>:environment:relay-prod
                          what-if:     repo:<owner>/<repo>:pull_request
  RoleAssignments       Least-privilege Azure roles, only at resource-group scope (never subscription scope):
                          deploy-dev SP, DEV RG only, and deploy-prod SP, PROD RG only: Contributor (deploys ACR,
                            identity, Log Analytics, ACI, budget), plus Role Based Access Control Administrator WITH AN
                            ABAC CONDITION that lets it write/delete role assignments ONLY for the AcrPull role (the
                            relay Bicep assigns AcrPull to the relay's managed identity on the registry). It cannot
                            grant Owner, Contributor, etc. The dev app never receives anything on the prod RG.
                          what-if SP, dev RG only: built-in Reader plus the custom role "wpiemu-relay-whatif"
                            (Microsoft.Resources/deployments whatIf/validate/read), because Reader alone does not
                            contain Microsoft.Resources/deployments/whatIf/action.
                        Owner and User Access Administrator are never assigned. A pre-existing role of a deploy app
                        on the other environment's resource group is reported as a failure (exit 1), not removed.
  GrantPromotion        AcrPull for the deploy-PROD SP (only) on the DEV registry so `az acr import` can promote an
                        image into the prod registry. The registry is created by the first infra deployment, so this
                        step is Skipped with a message until it exists: re-run `-Step GrantPromotion -Apply` afterwards.
  GitHub                Prints (and with -ConfigureGitHub applies with the gh CLI) the environments relay-dev (no
                        approval) and relay-prod (required reviewers, deployment branch restricted to main) and the
                        variables the workflows read. AZURE_CLIENT_ID of relay-dev is the dev app's, that of relay-prod
                        the prod app's.

Why one app per environment: the prod approval gate is a GitHub environment rule, but a token only needs its subject to
match a federated credential. If one app could deploy to both resource groups, a job in the unapproved relay-dev
environment could obtain that app's token and deploy into prod. Azure permissions must separate dev from prod, so each
deploy app can only touch its own resource group. Likewise the pull_request credential lives on its own what-if app
that only has Reader plus what-if on dev. -SkipPrIdentity creates no pull_request credential at all.

ABAC condition on Role Based Access Control Administrator (verbatim form, GUIDs from -DelegatedRoleIds, default AcrPull):
  ((!(ActionMatches{'Microsoft.Authorization/roleAssignments/write'})) OR
   (@Request[Microsoft.Authorization/roleAssignments:RoleDefinitionId] ForAnyOfAnyValues:GuidEquals {<guid>[, <guid>]}))
  AND
  ((!(ActionMatches{'Microsoft.Authorization/roleAssignments/delete'})) OR
   (@Resource[Microsoft.Authorization/roleAssignments:RoleDefinitionId] ForAnyOfAnyValues:GuidEquals {<guid>[, <guid>]}))
It is passed as one line with `--condition-version 2.0` (add/remove use different attribute sources, hence two clauses).

-Remove tears down what this script created: role assignments of all three apps inside the managed resource groups, the
custom role, the federated credentials and the app registrations. Resource groups are deleted only with
-RemoveResourceGroups and only when tagged managedBy=bootstrap. GitHub environments and variables are not removed (delete
them in the UI). Remove is a dry run too until -Apply.

.NOTES
Prerequisites for the human running it (the script never logs in):
  * PowerShell 7.0+ and the Azure CLI (az); the GitHub CLI (gh) only for -ConfigureGitHub.
  * `az login` (and `az account set --subscription <id>` or -SubscriptionId) as a user that has, on the subscription,
    Owner (or User Access Administrator plus Contributor), and in the Entra tenant Application Administrator,
    Cloud Application Administrator or Global Administrator (a plain member can also do it when the tenant lets users
    register applications; the script warns when it cannot tell). Creating the custom role needs Owner or
    User Access Administrator.
  * The resource providers checked by infra/relay/preflight.ps1 are registered (the same checks run first;
    `pwsh infra/relay/preflight.ps1 -Register` registers them). -SkipPreflight skips them.
  * For -ConfigureGitHub: `gh auth login` as a repository admin. Required reviewers for relay-prod need a plan that
    supports them (public repositories, or GitHub Team/Enterprise for private ones).

Exit codes: 0 ok (or a clean dry run), 1 a step failed, 2 preconditions or usage (nothing was changed).
Logs mask the subscription and tenant ids; -Json and the final "GitHub configuration" section print full values (they
are identifiers, not secrets, and must be copied). Federated credentials hold no secrets; there is no secret anywhere.

.PARAMETER Apply
Actually create/modify (or with -Remove, delete). Without it nothing is mutated.

.PARAMETER Force
Skip the confirmation prompt that -Apply shows. Required for non-interactive runs.

.PARAMETER Remove
Tear down instead of set up. Dry run unless -Apply.

.PARAMETER RemoveResourceGroups
With -Remove: also delete the resource groups (only when tagged managedBy=bootstrap). Requires -Remove.

.PARAMETER Step
One or more of All, ResourceGroups, Identity, FederatedCredentials, RoleAssignments, GrantPromotion, GitHub. Default All.

.PARAMETER SubscriptionId
Subscription id or name passed to az (--subscription). Default: the current az subscription.

.PARAMETER Location
Region of the resource groups. Default southcentralus.

.PARAMETER Repository
GitHub repository as owner/name. Default Overture-7421/wpi-emulator.

.PARAMETER DevEnvironment
GitHub environment name for dev. Default relay-dev.

.PARAMETER ProdEnvironment
GitHub environment name for prod. Default relay-prod.

.PARAMETER DevResourceGroup
Default rg-wpiemu-relay-dev.

.PARAMETER ProdResourceGroup
Default rg-wpiemu-relay-prod.

.PARAMETER AppNamePrefix
Prefix of the two deploy app registrations; the display names are <prefix>-dev and <prefix>-prod. Default wpiemu-relay-deploy.

.PARAMETER PrAppName
Display name of the pull-request (what-if) app registration. Default wpiemu-relay-whatif.

.PARAMETER SkipPrIdentity
Create no pull_request credential and no what-if app.

.PARAMETER DnsSuffix
DNS label suffix for the relay (lowercase letters, digits, dashes, 1-20 chars). Default: the first 6 hex characters of
the SHA-256 of the lower-case subscription id, so labels are unique and reproducible.

.PARAMETER AlertEmail
Budget alert email, published as RELAY_ALERT_EMAIL. Required with -ConfigureGitHub.

.PARAMETER DelegatedRoleIds
Role definition ids each deploy SP may assign/remove in its own resource group (the ABAC condition). Default: AcrPull 7f951dda-4ed3-4680-a7ca-43fe172d538d.

.PARAMETER ConfigureGitHub
Create the GitHub environments and variables with gh. Refused (exit 2) unless `gh auth status` succeeds.

.PARAMETER ProdReviewers
GitHub logins (or org/team-slug) that must approve relay-prod. Required with -ConfigureGitHub.

.PARAMETER DeploymentBranch
Branch relay-prod deployments are restricted to. Default main.

.PARAMETER SkipPreflight
Do not run the preflight checks first.

.PARAMETER SkipPermissionCheck
Do not check the signed-in principal's Azure role and Entra permissions.

.PARAMETER Json
Print one JSON document (steps, planned commands, GitHub configuration) instead of the text report.

.EXAMPLE
pwsh -NoProfile -File infra/relay/bootstrap.ps1

Dry run: prints everything it would create, changes nothing.

.EXAMPLE
pwsh -NoProfile -File infra/relay/bootstrap.ps1 -Apply -AlertEmail me@example.org

Creates the resource groups, apps, credentials and roles after a confirmation prompt, and prints the GitHub setup.

.EXAMPLE
pwsh -NoProfile -File infra/relay/bootstrap.ps1 -Apply -Force -ConfigureGitHub -ProdReviewers octocat -AlertEmail me@example.org

Also configures the GitHub environments and variables (needs `gh auth login`).

.EXAMPLE
pwsh -NoProfile -File infra/relay/bootstrap.ps1 -Apply -Step GrantPromotion

Run after the first dev infra deployment created the dev registry.

.EXAMPLE
pwsh -NoProfile -File infra/relay/bootstrap.ps1 -Remove

Dry run of the teardown. Add -Apply to execute it, and -RemoveResourceGroups to delete the resource groups too.
#>
#Requires -Version 7.0
[CmdletBinding()]
param(
    [switch]$Apply,
    [switch]$Force,
    [switch]$Remove,
    [switch]$RemoveResourceGroups,
    [ValidateSet('All', 'ResourceGroups', 'Identity', 'FederatedCredentials', 'RoleAssignments', 'GrantPromotion', 'GitHub')]
    [string[]]$Step,
    [string]$SubscriptionId,
    [ValidatePattern('^[a-z0-9]+$')]
    [string]$Location,
    [ValidatePattern('^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$')]
    [string]$Repository,
    [ValidatePattern('^[A-Za-z0-9_. -]+$')]
    [string]$DevEnvironment,
    [ValidatePattern('^[A-Za-z0-9_. -]+$')]
    [string]$ProdEnvironment,
    [ValidatePattern('^[-\w._()]+$')]
    [string]$DevResourceGroup,
    [ValidatePattern('^[-\w._()]+$')]
    [string]$ProdResourceGroup,
    [string]$AppNamePrefix,
    [string]$PrAppName,
    [switch]$SkipPrIdentity,
    [ValidatePattern('^[a-z0-9-]{1,20}$')]
    [string]$DnsSuffix,
    [string]$AlertEmail,
    [string[]]$DelegatedRoleIds,
    [switch]$ConfigureGitHub,
    [string[]]$ProdReviewers,
    [string]$DeploymentBranch,
    [switch]$SkipPreflight,
    [switch]$SkipPermissionCheck,
    [switch]$Json
)

# preflight.ps1 declares its own parameters, which a dot-source would bind over ours: capture ours first.
$script:CliOptions = @{} + $PSBoundParameters
$script:DotSourced = $MyInvocation.InvocationName -eq '.'

Set-StrictMode -Version Latest

. (Join-Path $PSScriptRoot 'preflight.ps1')

$script:BuiltInRoles = @{
    Contributor = 'b24988ac-6180-42a0-ab88-20f7382dd24c'
    Reader      = 'acdd72a7-3385-48ef-bd42-f606fba81ae7'
    RbacAdmin   = 'f58310d9-a9f6-439a-9e8d-f62e7b41a168'
    AcrPull     = '7f951dda-4ed3-4680-a7ca-43fe172d538d'
}
$script:WhatIfRoleName = 'wpiemu-relay-whatif'
$script:WhatIfRoleActions = @(
    'Microsoft.Resources/deployments/whatIf/action',
    'Microsoft.Resources/deployments/validate/action',
    'Microsoft.Resources/deployments/read',
    'Microsoft.Resources/deployments/operations/read'
)
$script:OidcIssuer = 'https://token.actions.githubusercontent.com'
$script:OidcAudience = 'api://AzureADTokenExchange'
$script:JsonToken = '{{JSONFILE}}'
$script:StepOrder = @('ResourceGroups', 'Identity', 'FederatedCredentials', 'RoleAssignments', 'GrantPromotion', 'GitHub')
$script:StepDependencies = @{
    ResourceGroups       = @()
    Identity             = @()
    FederatedCredentials = @('Identity')
    RoleAssignments      = @('Identity', 'ResourceGroups')
    GrantPromotion       = @('Identity', 'ResourceGroups')
    GitHub               = @('Identity')
}
$script:CommonParameters = @('Verbose', 'Debug', 'ErrorAction', 'WarningAction', 'InformationAction', 'ErrorVariable',
    'WarningVariable', 'InformationVariable', 'OutVariable', 'OutBuffer', 'PipelineVariable', 'ProgressAction')

# Tests replace these. GhRunner takes ([string[]]$GhArgs) and returns ExitCode/Output like AzRunner (from preflight.ps1).
$script:GhRunner = $null
$script:Sleep = { param([int]$Seconds) Start-Sleep -Seconds $Seconds }
$script:ConfirmRunner = $null
$script:ScratchDir = [System.IO.Path]::GetTempPath()

# ---- small helpers --------------------------------------------------------------------------------------------------

function Format-MaskedId {
    param([AllowNull()][string]$Id)
    if ([string]::IsNullOrEmpty($Id) -or $Id.Length -lt 12) { return $Id }
    '{0}...{1}' -f $Id.Substring(0, 8), $Id.Substring($Id.Length - 4)
}

function Protect-Text {
    param([AllowNull()][string]$Text)
    if ($null -eq $Text) { return '' }
    foreach ($id in @($script:Run.SubscriptionId, $script:Run.TenantId)) {
        if ($id -and $id.Length -ge 12) { $Text = $Text -replace [regex]::Escape($id), (Format-MaskedId $id) }
    }
    $Text
}

function Write-Log {
    param([string]$Text, [string]$Color = '')
    if ($script:Run.Options.Json) { return }
    if ($Color) { Write-Host (Protect-Text $Text) -ForegroundColor $Color } else { Write-Host (Protect-Text $Text) }
}

function Format-Command {
    param([string]$Tool, [string[]]$ToolArgs, [string]$JsonBody)
    $quote = { param($s) if ($s -match '^[A-Za-z0-9_./:=@,-]+$') { $s } else { "'" + ($s -replace "'", "''") + "'" } }
    $parts = foreach ($a in $ToolArgs) {
        if ($JsonBody -and $a -eq $script:JsonToken) { '-' }
        elseif ($JsonBody -and $a.Contains($script:JsonToken)) { & $quote $JsonBody }
        else { & $quote $a }
    }
    $line = "$Tool " + ($parts -join ' ')
    if ($JsonBody -and $Tool -eq 'gh') { return (& $quote $JsonBody) + " | $line" }
    $line
}
function Test-Placeholder {
    param([AllowNull()][string]$Value)
    [string]::IsNullOrEmpty($Value) -or $Value.StartsWith('<')
}

function Get-DefaultDnsSuffix {
    param([Parameter(Mandatory)][string]$SubscriptionId)
    $bytes = [System.Text.Encoding]::UTF8.GetBytes($SubscriptionId.Trim().ToLowerInvariant())
    $hash = [System.Security.Cryptography.SHA256]::HashData($bytes)
    (([System.BitConverter]::ToString($hash) -replace '-', '').ToLowerInvariant()).Substring(0, 6)
}

function New-RoleAssignmentCondition {
    param([Parameter(Mandatory)][string[]]$RoleIds)
    $ids = ($RoleIds | ForEach-Object { $_.Trim().ToLowerInvariant() }) -join ', '
    "((!(ActionMatches{'Microsoft.Authorization/roleAssignments/write'})) OR (@Request[Microsoft.Authorization/roleAssignments:RoleDefinitionId] ForAnyOfAnyValues:GuidEquals {$ids})) AND ((!(ActionMatches{'Microsoft.Authorization/roleAssignments/delete'})) OR (@Resource[Microsoft.Authorization/roleAssignments:RoleDefinitionId] ForAnyOfAnyValues:GuidEquals {$ids}))"
}

# Azure may return the condition reformatted: equal text, or the same GUID set under the same two actions, is the same condition.
function Test-ConditionEquivalent {
    param([AllowNull()][string]$A, [AllowNull()][string]$B)
    $norm = { param($s) if ($null -eq $s) { '' } else { ($s -replace '\s+', '').ToLowerInvariant() } }
    $x = & $norm $A
    $y = & $norm $B
    if ($x -eq $y) { return $true }
    if (-not $x -or -not $y) { return $false }
    $guids = { param($s) @([regex]::Matches($s, '[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}') | ForEach-Object { $_.Value } | Sort-Object -Unique) -join ',' }
    foreach ($needle in 'roleassignments/write', 'roleassignments/delete', 'foranyofanyvalues:guidequals') {
        if (-not $x.Contains($needle) -or -not $y.Contains($needle)) { return $false }
    }
    (& $guids $x) -eq (& $guids $y)
}

# ---- options --------------------------------------------------------------------------------------------------------

function Resolve-BootstrapOptions {
    param([hashtable]$Bound = @{})

    $o = @{
        Apply = $false; Force = $false; Remove = $false; RemoveResourceGroups = $false
        Step = @('All'); SubscriptionId = ''; Location = 'southcentralus'
        Repository = 'Overture-7421/wpi-emulator'
        DevEnvironment = 'relay-dev'; ProdEnvironment = 'relay-prod'
        DevResourceGroup = 'rg-wpiemu-relay-dev'; ProdResourceGroup = 'rg-wpiemu-relay-prod'
        AppNamePrefix = 'wpiemu-relay-deploy'; PrAppName = 'wpiemu-relay-whatif'
        SkipPrIdentity = $false
        DnsSuffix = ''; AlertEmail = ''
        DelegatedRoleIds = @($script:BuiltInRoles.AcrPull)
        ConfigureGitHub = $false; ProdReviewers = @(); DeploymentBranch = 'main'
        SkipPreflight = $false; SkipPermissionCheck = $false; Json = $false
    }
    foreach ($k in $Bound.Keys) {
        if ($script:CommonParameters -contains $k) { continue }
        if (-not $o.ContainsKey($k)) { throw [System.ArgumentException]::new("Unknown option '$k'.") }
        $v = $Bound[$k]
        if ($v -is [System.Management.Automation.SwitchParameter]) { $v = $v.IsPresent }
        if ($o[$k] -is [array]) { $v = @($v) }
        $o[$k] = $v
    }

    if ($o.RemoveResourceGroups -and -not $o.Remove) { throw [System.ArgumentException]::new('-RemoveResourceGroups requires -Remove.') }
    if ($o.AppNamePrefix -eq $o.PrAppName -or "$($o.AppNamePrefix)-dev" -eq $o.PrAppName -or "$($o.AppNamePrefix)-prod" -eq $o.PrAppName) { throw [System.ArgumentException]::new('-PrAppName must differ from the deploy app names.') }
    if ($o.Remove -and $o.ConfigureGitHub) { throw [System.ArgumentException]::new('-ConfigureGitHub cannot be combined with -Remove (delete GitHub environments and variables in the UI).') }
    if ($o.DevResourceGroup -eq $o.ProdResourceGroup) { throw [System.ArgumentException]::new('Dev and prod must use different resource groups.') }
    if ($o.DevEnvironment -eq $o.ProdEnvironment) { throw [System.ArgumentException]::new('Dev and prod must use different GitHub environments.') }
    foreach ($id in $o.DelegatedRoleIds) {
        if ($id -notmatch '^[0-9a-fA-F]{8}-([0-9a-fA-F]{4}-){3}[0-9a-fA-F]{12}$') { throw [System.ArgumentException]::new("DelegatedRoleIds entry '$id' is not a GUID.") }
    }
    if ($o.DelegatedRoleIds.Count -eq 0) { throw [System.ArgumentException]::new('DelegatedRoleIds must not be empty.') }
    $o.Step = @($o.Step)
    if ($o.Step.Count -eq 0 -or $o.Step -contains 'All') { $o.Step = @('All') }
    $o
}

function Get-SelectedSteps {
    if ($script:Run.Options.Step -contains 'All') { return @($script:StepOrder) }
    @($script:StepOrder | Where-Object { $script:Run.Options.Step -contains $_ })
}

# ---- tool invocation (the only place that touches az and gh) --------------------------------------------------------

function Invoke-Gh {
    param([Parameter(Mandatory)][string[]]$GhArgs)

    if ($null -ne $script:GhRunner) { return & $script:GhRunner $GhArgs }
    $all = & gh @GhArgs 2>&1
    $code = $LASTEXITCODE
    $out = @($all | Where-Object { $_ -isnot [System.Management.Automation.ErrorRecord] })
    $err = @($all | Where-Object { $_ -is [System.Management.Automation.ErrorRecord] } | ForEach-Object { $_.ToString() })
    [pscustomobject]@{ ExitCode = $code; Output = (($out + $err) -join "`n"); StdOut = ($out -join "`n") }
}

function Assert-ReadOnlyCall {
    param([string]$Tool, [string[]]$ToolArgs)

    $words = [System.Collections.Generic.List[string]]::new()
    foreach ($a in $ToolArgs) { if ($a.StartsWith('-')) { break }; $words.Add($a) }
    $verbs = 'create', 'update', 'delete', 'set', 'import', 'register', 'purge', 'add', 'remove', 'login', 'logout', 'assign', 'deploy', 'cancel', 'restore'
    $bad = @($words | Where-Object { $verbs -contains $_ })
    $flags = ($ToolArgs | Where-Object { $_.StartsWith('-') }) -join ' '
    $mutatingHttp = $false
    for ($i = 0; $i -lt $ToolArgs.Count - 1; $i++) {
        if ($ToolArgs[$i] -in '--method', '-X', '-m' -and $ToolArgs[$i + 1] -notmatch '^(?i)GET$') { $mutatingHttp = $true }
    }
    if ($Tool -eq 'gh' -and $flags -match '(^| )(-f|-F|--field|--raw-field|--input)( |$)') { $mutatingHttp = $true }
    if ($bad.Count -gt 0 -or $mutatingHttp) {
        throw [System.InvalidOperationException]::new("Refusing to run a mutating $Tool command as a read: $Tool $($ToolArgs -join ' ')")
    }
}

function Get-SubscriptionArgs {
    param([string[]]$ToolArgs)
    if ($script:Run.Options.SubscriptionId -and $ToolArgs.Count -gt 0 -and $ToolArgs[0] -in 'group', 'role', 'acr') {
        return @('--subscription', $script:Run.Options.SubscriptionId)
    }
    @()
}

# Runs az/gh. Mutating calls are recorded and skipped in dry-run; JSON bodies reach the CLI as a file replacing {{JSONFILE}}.
function Invoke-Tool {
    param(
        [Parameter(Mandatory)][ValidateSet('az', 'gh')][string]$Tool,
        [Parameter(Mandatory)][string[]]$ToolArgs,
        [switch]$Mutating,
        [string]$JsonBody = '',
        [string]$RetryPattern = '',
        [int]$MaxAttempts = 1
    )

    if ($Tool -eq 'az') { $ToolArgs = @($ToolArgs) + @(Get-SubscriptionArgs $ToolArgs) }
    if (-not $Mutating) { Assert-ReadOnlyCall -Tool $Tool -ToolArgs $ToolArgs }

    $display = Format-Command -Tool $Tool -ToolArgs $ToolArgs -JsonBody $JsonBody
    if ($Mutating -and -not $script:Run.Options.Apply) {
        $script:Run.Planned.Add([pscustomobject]@{ Tool = $Tool; Args = $ToolArgs; Json = $JsonBody; Command = (Protect-Text $display) })
        Write-Log "  [dry-run] would run: $display" 'DarkYellow'
        return [pscustomobject]@{ ExitCode = 0; Output = ''; StdOut = ''; DryRun = $true }
    }
    if ($Mutating) {
        $script:Run.Executed.Add([pscustomobject]@{ Tool = $Tool; Args = $ToolArgs; Json = $JsonBody })
        Write-Log "  running: $display" 'DarkGray'
    }

    $file = ''
    try {
        $runArgs = $ToolArgs
        if ($JsonBody) {
            $file = Join-Path $script:ScratchDir ('wpiemu-bootstrap-{0}.json' -f [guid]::NewGuid().ToString('N'))
            [System.IO.File]::WriteAllText($file, $JsonBody, [System.Text.UTF8Encoding]::new($false))
            $runArgs = @($ToolArgs | ForEach-Object { $_.Replace($script:JsonToken, $file) })
        }
        $attempt = 0
        while ($true) {
            $attempt++
            $r = if ($Tool -eq 'az') { Invoke-Az -AzArgs $runArgs } else { Invoke-Gh -GhArgs $runArgs }
            if ($r.ExitCode -eq 0 -or $attempt -ge $MaxAttempts -or -not $RetryPattern -or $r.Output -notmatch $RetryPattern) { return $r }
            Write-Log "  not ready yet (directory propagation), retrying in 10 s ($attempt/$MaxAttempts)" 'DarkGray'
            & $script:Sleep 10
        }
    }
    finally {
        if ($file -and (Test-Path -LiteralPath $file)) { Remove-Item -LiteralPath $file -Force -ErrorAction SilentlyContinue }
    }
}

function ConvertFrom-ToolResult {
    param($Result)
    if ($Result.ExitCode -ne 0) {
        if (Test-AzAuthFailure $Result.Output) { throw [System.InvalidOperationException]::new("AZ_AUTH: $($Result.Output.Trim())") }
        return [pscustomobject]@{ Ok = $false; Data = $null; Error = $Result.Output.Trim() }
    }
    $text = if ($Result.PSObject.Properties['StdOut']) { $Result.StdOut } else { $Result.Output }
    $data = $null
    if (-not [string]::IsNullOrWhiteSpace($text)) {
        try { $data = $text | ConvertFrom-Json -NoEnumerate -ErrorAction Stop }
        catch { return [pscustomobject]@{ Ok = $false; Data = $null; Error = "unparseable output: $text" } }
    }
    [pscustomobject]@{ Ok = $true; Data = $data; Error = '' }
}

function Read-Az { param([string[]]$AzArgs) ConvertFrom-ToolResult (Invoke-Tool -Tool az -ToolArgs $AzArgs) }
function Read-Gh { param([string[]]$GhArgs) ConvertFrom-ToolResult (Invoke-Tool -Tool gh -ToolArgs $GhArgs) }

function Read-AzOrThrow {
    param([string[]]$AzArgs)
    $r = Read-Az $AzArgs
    if (-not $r.Ok) { throw [System.InvalidOperationException]::new("az $($AzArgs[0..([Math]::Min(3, $AzArgs.Count - 1))] -join ' ') failed: $($r.Error)") }
    $r.Data
}

function Invoke-Mutation {
    param(
        [Parameter(Mandatory)][ValidateSet('az', 'gh')][string]$Tool,
        [Parameter(Mandatory)][string[]]$ToolArgs,
        [string]$JsonBody = '',
        [string]$RetryPattern = '',
        [int]$MaxAttempts = 1
    )
    $r = Invoke-Tool -Tool $Tool -ToolArgs $ToolArgs -Mutating -JsonBody $JsonBody -RetryPattern $RetryPattern -MaxAttempts $MaxAttempts
    if ($r.ExitCode -ne 0) {
        $short = ($r.Output.Trim() -split "`n" | Select-Object -First 3) -join ' '
        throw [System.InvalidOperationException]::new("$Tool $($ToolArgs[0..([Math]::Min(2, $ToolArgs.Count - 1))] -join ' ') failed (exit $($r.ExitCode)): $short")
    }
    $r
}

function Get-MutationData {
    param($Result)
    if ($Result.PSObject.Properties['DryRun'] -and $Result.DryRun) { return $null }
    (ConvertFrom-ToolResult $Result).Data
}

# ---- run state and reporting ----------------------------------------------------------------------------------------

function Add-Item {
    param([string]$StepName, [string]$Name, [string]$Status, [string]$Detail = '')
    $item = [pscustomobject]@{ Step = $StepName; Item = $Name; Status = $Status; Detail = $Detail }
    $script:Run.Items.Add($item)
    $color = switch ($Status) { 'Failed' { 'Red' } 'Skipped' { 'Yellow' } 'Exists' { 'Gray' } default { 'Green' } }
    $suffix = if ($Detail) { " ($Detail)" } else { '' }
    Write-Log ("  [{0}] {1}{2}" -f $Status, $Name, $suffix) $color
}

function Add-Warning {
    param([string]$Text)
    $script:Run.Warnings.Add($Text)
    Write-Log "  WARN: $Text" 'Yellow'
}

function Get-ResourceGroupSpecs {
    $o = $script:Run.Options
    @(
        [pscustomobject]@{ Name = $o.DevResourceGroup; Env = 'dev'; Scope = "/subscriptions/$($script:Run.SubscriptionId)/resourceGroups/$($o.DevResourceGroup)" },
        [pscustomobject]@{ Name = $o.ProdResourceGroup; Env = 'prod'; Scope = "/subscriptions/$($script:Run.SubscriptionId)/resourceGroups/$($o.ProdResourceGroup)" }
    )
}

function Get-AppConfigs {
    $o = $script:Run.Options
    $apps = @(
        [pscustomobject]@{
            Key = 'dev'; DisplayName = "$($o.AppNamePrefix)-dev"
            Credentials = @([pscustomobject]@{ Name = "github-$($o.DevEnvironment)"; Subject = "repo:$($o.Repository):environment:$($o.DevEnvironment)" })
        },
        [pscustomobject]@{
            Key = 'prod'; DisplayName = "$($o.AppNamePrefix)-prod"
            Credentials = @([pscustomobject]@{ Name = "github-$($o.ProdEnvironment)"; Subject = "repo:$($o.Repository):environment:$($o.ProdEnvironment)" })
        }
    )
    if (-not $o.SkipPrIdentity) {
        $apps += [pscustomobject]@{
            Key = 'pr'; DisplayName = $o.PrAppName
            Credentials = @([pscustomobject]@{ Name = 'github-pull-request'; Subject = "repo:$($o.Repository):pull_request" })
        }
    }
    $apps
}

# The only resource group an app may hold roles in: dev and pr in the dev RG, prod in the prod RG.
function Get-AppHomeScope {
    param([Parameter(Mandatory)][string]$AppKey)
    $rgs = @(Get-ResourceGroupSpecs)
    if ($AppKey -eq 'prod') { $rgs[1].Scope } else { $rgs[0].Scope }
}

function Test-UnderScope {
    param([string]$Scope, [string]$Parent)
    $Scope -ieq $Parent -or $Scope.StartsWith($Parent.TrimEnd('/') + '/', [System.StringComparison]::OrdinalIgnoreCase)
}

# The one cross-environment grant that is allowed: AcrPull on the dev registry for the prod app (az acr import).
function Test-PromotionGrant {
    param([string]$AppKey, [string]$Scope, [string]$RoleId)
    $dev = (Get-ResourceGroupSpecs)[0]
    $AppKey -eq 'prod' -and $RoleId -ieq $script:BuiltInRoles.AcrPull -and
    $Scope -match '(?i)/providers/Microsoft\.ContainerRegistry/registries/[^/]+$' -and (Test-UnderScope $Scope $dev.Scope)
}

# Hard guard on every assignment this script creates: the dev app can never receive a prod-scoped role, and the
# prod app only its own RG plus the promotion grant.
function Assert-AssignmentAllowed {
    param([string]$AppKey, [string]$Scope, [string]$RoleId)
    if (Test-UnderScope $Scope (Get-AppHomeScope $AppKey)) { return }
    if (Test-PromotionGrant $AppKey $Scope $RoleId) { return }
    throw [System.InvalidOperationException]::new("Refusing to assign '$RoleId' to the $AppKey app at ${Scope}: outside its own environment.")
}

function Get-AppState {
    param([Parameter(Mandatory)]$App)

    if ($script:Run.Apps.ContainsKey($App.Key)) { return $script:Run.Apps[$App.Key] }
    $state = @{ Key = $App.Key; DisplayName = $App.DisplayName; Exists = $false; Planned = $false; ObjectId = ''; ClientId = ''; SpExists = $false; SpObjectId = '' }

    $apps = @(Read-AzOrThrow @('ad', 'app', 'list', '--display-name', $App.DisplayName, '-o', 'json') | Where-Object { $null -ne $_ })
    $exact = @($apps | Where-Object { (Get-Prop $_ 'displayName' '') -ceq $App.DisplayName })
    if ($exact.Count -gt 1) {
        throw [System.InvalidOperationException]::new("$($exact.Count) app registrations are named '$($App.DisplayName)'; refusing to guess. Delete the duplicates or pass a different -AppNamePrefix/-PrAppName.")
    }
    if ($exact.Count -eq 1) {
        $state.Exists = $true
        $state.ObjectId = [string](Get-Prop $exact[0] 'id' '')
        $state.ClientId = [string](Get-Prop $exact[0] 'appId' '')
        if ([string]::IsNullOrEmpty($state.ClientId)) { throw [System.InvalidOperationException]::new("App '$($App.DisplayName)' has no appId in the az output.") }
        $secrets = @(Get-Prop $exact[0] 'passwordCredentials' @())
        if ($secrets.Count -gt 0) { Add-Warning "App '$($App.DisplayName)' has $($secrets.Count) client secret(s). The pipelines use OIDC only: delete them (az ad app credential delete)." }
        $sps = @(Read-AzOrThrow @('ad', 'sp', 'list', '--filter', "appId eq '$($state.ClientId)'", '-o', 'json') | Where-Object { $null -ne $_ })
        if ($sps.Count -ge 1) {
            $state.SpExists = $true
            $state.SpObjectId = [string](Get-Prop $sps[0] 'id' '')
        }
    }
    $script:Run.Apps[$App.Key] = $state
    $state
}

function Set-PlannedApp {
    param($State, [switch]$AppToo)
    if ($AppToo) { $State.ObjectId = "<$($State.Key)-app-object-id>"; $State.ClientId = "<$($State.Key)-client-id>" }
    $State.SpObjectId = "<$($State.Key)-sp-object-id>"
    $State.Planned = $true
}

function Test-AppUsable {
    param($State)
    $State.Exists -or $State.Planned
}

function Assert-AppUsable {
    param($State)
    if (-not (Test-AppUsable $State)) {
        throw [System.InvalidOperationException]::new("App registration '$($State.DisplayName)' does not exist yet: run the Identity step first.")
    }
    if (-not $State.SpExists -and -not $State.Planned) {
        throw [System.InvalidOperationException]::new("Service principal of '$($State.DisplayName)' does not exist yet: run the Identity step first.")
    }
}

# ---- steps: setup ---------------------------------------------------------------------------------------------------

function Step-ResourceGroups {
    $o = $script:Run.Options
    foreach ($rg in Get-ResourceGroupSpecs) {
        $tags = [ordered]@{ app = 'wpiemu-relay'; environment = $rg.Env; managedBy = 'bootstrap' }
        $exists = Read-AzOrThrow @('group', 'exists', '--name', $rg.Name)
        if ($exists -ne $true) {
            $tagArgs = @($tags.GetEnumerator() | ForEach-Object { "$($_.Key)=$($_.Value)" })
            $null = Invoke-Mutation az (@('group', 'create', '--name', $rg.Name, '--location', $o.Location, '--tags') + $tagArgs + @('-o', 'none'))
            Add-Item 'ResourceGroups' $rg.Name $(if ($o.Apply) { 'Created' } else { 'WouldCreate' }) $o.Location
            continue
        }
        $current = Read-AzOrThrow @('group', 'show', '--name', $rg.Name, '-o', 'json')
        $currentTags = Get-Prop $current 'tags' $null
        $drift = @($tags.GetEnumerator() | Where-Object {
                $have = if ($null -ne $currentTags) { Get-Prop $currentTags $_.Key '' } else { '' }
                $have -ne $_.Value
            } | ForEach-Object { "tags.$($_.Key)=$($_.Value)" })
        $loc = [string](Get-Prop $current 'location' '')
        if ($loc -and $loc -ne $o.Location) { Add-Warning "Resource group $($rg.Name) is in '$loc', not '$($o.Location)'; a resource group cannot move region." }
        if ($drift.Count -gt 0) {
            $null = Invoke-Mutation az (@('group', 'update', '--name', $rg.Name, '--set') + $drift + @('-o', 'none'))
            Add-Item 'ResourceGroups' $rg.Name $(if ($o.Apply) { 'Updated' } else { 'WouldUpdate' }) 'tags'
        }
        else {
            Add-Item 'ResourceGroups' $rg.Name 'Exists'
        }
    }
}

function Step-Identity {
    $o = $script:Run.Options
    $propagation = 'does not exist|not found|ResourceNotFound|referenced'
    foreach ($app in Get-AppConfigs) {
        $state = Get-AppState $app
        if (-not $state.Exists) {
            $r = Invoke-Mutation az @('ad', 'app', 'create', '--display-name', $app.DisplayName, '--sign-in-audience', 'AzureADMyOrg', '-o', 'json')
            $data = Get-MutationData $r
            if ($null -eq $data) { Set-PlannedApp $state -AppToo }
            else {
                $state.Exists = $true
                $state.ObjectId = [string](Get-Prop $data 'id' '')
                $state.ClientId = [string](Get-Prop $data 'appId' '')
                if (-not $state.ClientId) { throw [System.InvalidOperationException]::new('az ad app create returned no appId.') }
            }
            Add-Item 'Identity' "app $($app.DisplayName)" $(if ($o.Apply) { 'Created' } else { 'WouldCreate' })
        }
        else {
            Add-Item 'Identity' "app $($app.DisplayName)" 'Exists' $state.ClientId
        }

        if (-not $state.SpExists) {
            $r = Invoke-Mutation az @('ad', 'sp', 'create', '--id', $state.ClientId, '-o', 'json') -RetryPattern $propagation -MaxAttempts 6
            $data = Get-MutationData $r
            if ($null -eq $data) { Set-PlannedApp $state }
            else {
                $state.SpExists = $true
                $state.SpObjectId = [string](Get-Prop $data 'id' '')
                if (-not $state.SpObjectId) { throw [System.InvalidOperationException]::new('az ad sp create returned no id.') }
            }
            Add-Item 'Identity' "service principal $($app.DisplayName)" $(if ($o.Apply) { 'Created' } else { 'WouldCreate' })
        }
        else {
            Add-Item 'Identity' "service principal $($app.DisplayName)" 'Exists' $state.SpObjectId
        }
    }
}

function Get-CredentialBody {
    param($Spec)
    ([ordered]@{
            name        = $Spec.Name
            issuer      = $script:OidcIssuer
            subject     = $Spec.Subject
            description = 'WPI Emulator relay pipeline (managed by infra/relay/bootstrap.ps1)'
            audiences   = @($script:OidcAudience)
        } | ConvertTo-Json -Compress)
}

function Step-FederatedCredentials {
    $o = $script:Run.Options
    $propagation = 'does not exist|not found|ResourceNotFound|referenced'
    foreach ($app in Get-AppConfigs) {
        $state = Get-AppState $app
        Assert-AppUsable $state
        $existing = @()
        if (-not $state.Planned) {
            $existing = @(Read-AzOrThrow @('ad', 'app', 'federated-credential', 'list', '--id', $state.ClientId, '-o', 'json') | Where-Object { $null -ne $_ })
        }
        foreach ($spec in $app.Credentials) {
            $label = "$($app.DisplayName): $($spec.Name)"
            $byName = @($existing | Where-Object { (Get-Prop $_ 'name' '') -ceq $spec.Name })
            $bySubject = @($existing | Where-Object { (Get-Prop $_ 'subject' '') -ceq $spec.Subject -and (Get-Prop $_ 'issuer' '') -ceq $script:OidcIssuer })
            $found = if ($byName.Count -gt 0) { $byName[0] } elseif ($bySubject.Count -gt 0) { $bySubject[0] } else { $null }
            if ($null -eq $found) {
                $null = Invoke-Mutation az @('ad', 'app', 'federated-credential', 'create', '--id', $state.ClientId, '--parameters', "@$($script:JsonToken)") `
                    -JsonBody (Get-CredentialBody $spec) -RetryPattern $propagation -MaxAttempts 6
                Add-Item 'FederatedCredentials' $label $(if ($o.Apply) { 'Created' } else { 'WouldCreate' }) $spec.Subject
                continue
            }
            $audiences = @(Get-Prop $found 'audiences' @())
            $same = (Get-Prop $found 'issuer' '') -ceq $script:OidcIssuer -and (Get-Prop $found 'subject' '') -ceq $spec.Subject -and ($audiences -contains $script:OidcAudience)
            if ($same) {
                Add-Item 'FederatedCredentials' $label 'Exists' $spec.Subject
                continue
            }
            $foundName = [string](Get-Prop $found 'name' $spec.Name)
            $body = [ordered]@{ name = $foundName; issuer = $script:OidcIssuer; subject = $spec.Subject; audiences = @($script:OidcAudience) } | ConvertTo-Json -Compress
            $null = Invoke-Mutation az @('ad', 'app', 'federated-credential', 'update', '--id', $state.ClientId, '--federated-credential-id', $foundName, '--parameters', "@$($script:JsonToken)") -JsonBody $body
            Add-Item 'FederatedCredentials' $label $(if ($o.Apply) { 'Updated' } else { 'WouldUpdate' }) "subject $($found.subject) -> $($spec.Subject)"
        }
    }
}

# Role assignments each app is supposed to have. Role is a built-in role GUID or, for IsCustom, the custom role name.
function Get-RoleSpecs {
    param([Parameter(Mandatory)][string]$AppKey)

    $o = $script:Run.Options
    $rgs = @(Get-ResourceGroupSpecs)
    $condition = New-RoleAssignmentCondition -RoleIds $o.DelegatedRoleIds
    $specs = [System.Collections.Generic.List[object]]::new()
    $isPr = $AppKey -eq 'pr'
    if (-not $isPr) {
        $rg = if ($AppKey -eq 'prod') { $rgs[1] } else { $rgs[0] }
        $specs.Add([pscustomobject]@{ Label = "Contributor on $($rg.Name)"; Role = $script:BuiltInRoles.Contributor; IsCustom = $false; Scope = $rg.Scope; Condition = '' })
        $specs.Add([pscustomobject]@{ Label = "Role Based Access Control Administrator (AcrPull only) on $($rg.Name)"; Role = $script:BuiltInRoles.RbacAdmin; IsCustom = $false; Scope = $rg.Scope; Condition = $condition })
    }
    else {
        $dev = $rgs[0]
        $specs.Add([pscustomobject]@{ Label = "Reader on $($dev.Name)"; Role = $script:BuiltInRoles.Reader; IsCustom = $false; Scope = $dev.Scope; Condition = '' })
        $specs.Add([pscustomobject]@{ Label = "$($script:WhatIfRoleName) on $($dev.Name)"; Role = $script:WhatIfRoleName; IsCustom = $true; Scope = $dev.Scope; Condition = '' })
    }
    $specs.ToArray()
}

function Get-WhatIfRoleBody {
    param([string]$Scope, [string]$Id = '')
    $def = [ordered]@{
        Name            = $script:WhatIfRoleName
        IsCustom        = $true
        Description     = 'Lets the WPI Emulator relay pull-request job run az deployment group what-if/validate. Used together with Reader. Created by infra/relay/bootstrap.ps1.'
        Actions         = @($script:WhatIfRoleActions)
        NotActions      = @()
        AssignableScopes = @($Scope)
    }
    if ($Id) { $def['Id'] = $Id }
    $def | ConvertTo-Json -Compress
}

function Get-RoleAssignmentsOf {
    param($State)
    if ($State.Planned -or -not $State.SpExists) { return @() }
    @(Read-AzOrThrow @('role', 'assignment', 'list', '--assignee', $State.SpObjectId, '--all', '-o', 'json') | Where-Object { $null -ne $_ })
}

function Find-RoleAssignment {
    param([object[]]$Assignments, $Spec)
    @($Assignments | Where-Object {
            $sameScope = ([string](Get-Prop $_ 'scope' '')).TrimEnd('/') -ieq $Spec.Scope.TrimEnd('/')
            $defId = [string](Get-Prop $_ 'roleDefinitionId' '')
            $defName = [string](Get-Prop $_ 'roleDefinitionName' '')
            $sameRole = if ($Spec.IsCustom) { $defName -ieq $Spec.Role } else { $defId -match "(?i)/$([regex]::Escape($Spec.Role))$" }
            $sameScope -and $sameRole
        })
}

function New-AssignmentArgs {
    param($State, $Spec)
    $a = @('role', 'assignment', 'create', '--assignee-object-id', $State.SpObjectId, '--assignee-principal-type', 'ServicePrincipal', '--role', $Spec.Role, '--scope', $Spec.Scope)
    if ($Spec.Condition) { $a += @('--condition', $Spec.Condition, '--condition-version', '2.0') }
    $a + @('-o', 'none')
}

function Set-RoleAssignment {
    param([Parameter(Mandatory)][string]$AppKey, $State, $Spec, [object[]]$Assignments, [string]$StepName = 'RoleAssignments')

    $o = $script:Run.Options
    Assert-AssignmentAllowed -AppKey $AppKey -Scope $Spec.Scope -RoleId $Spec.Role
    $propagation = 'PrincipalNotFound|does not exist in the directory|Cannot find'
    $found = @(Find-RoleAssignment $Assignments $Spec)
    $match = @($found | Where-Object { Test-ConditionEquivalent (Get-Prop $_ 'condition' '') $Spec.Condition })
    if ($match.Count -gt 0) {
        Add-Item $StepName $Spec.Label 'Exists'
        return
    }
    if ($found.Count -gt 0) {
        foreach ($stale in $found) {
            $null = Invoke-Mutation az @('role', 'assignment', 'delete', '--ids', [string](Get-Prop $stale 'id' ''), '-o', 'none')
        }
        $null = Invoke-Mutation az (New-AssignmentArgs $State $Spec) -RetryPattern $propagation -MaxAttempts 6
        Add-Item $StepName $Spec.Label $(if ($o.Apply) { 'Updated' } else { 'WouldUpdate' }) 'condition differed'
        return
    }
    $null = Invoke-Mutation az (New-AssignmentArgs $State $Spec) -RetryPattern $propagation -MaxAttempts 6
    Add-Item $StepName $Spec.Label $(if ($o.Apply) { 'Created' } else { 'WouldCreate' })
}

function Step-RoleAssignments {
    $o = $script:Run.Options
    $rgs = @(Get-ResourceGroupSpecs)
    foreach ($app in Get-AppConfigs) {
        $state = Get-AppState $app
        Assert-AppUsable $state
        $specs = @(Get-RoleSpecs -AppKey $app.Key)
        $assignments = @(Get-RoleAssignmentsOf $state)

        $customSpec = @($specs | Where-Object { $_.IsCustom })
        if ($customSpec.Count -gt 0) { Set-WhatIfRoleDefinition -Scope $customSpec[0].Scope }

        foreach ($spec in $specs) { Set-RoleAssignment -AppKey $app.Key -State $state -Spec $spec -Assignments $assignments }

        $homeScope = Get-AppHomeScope $app.Key
        foreach ($a in $assignments) {
            $scope = [string](Get-Prop $a 'scope' '')
            $roleId = ([string](Get-Prop $a 'roleDefinitionId' '')) -replace '^.*/', ''
            $roleName = Get-Prop $a 'roleDefinitionName' '?'
            $inHome = Test-UnderScope $scope $homeScope
            $inManaged = @($rgs | Where-Object { Test-UnderScope $scope $_.Scope }).Count -gt 0
            $isExpected = @($specs | Where-Object { @(Find-RoleAssignment @($a) $_).Count -gt 0 }).Count -gt 0
            if ($inManaged -and -not $inHome -and -not (Test-PromotionGrant $app.Key $scope $roleId)) {
                Add-Item 'RoleAssignments' "$($app.DisplayName): cross-environment role '$roleName' at $scope" 'Failed' 'this app must only hold roles in its own environment: remove the assignment (az role assignment delete --ids <id>)'
                $null = $script:Run.Failed.Add('RoleAssignments')
            }
            elseif ($inHome -and -not $isExpected) {
                Add-Warning "$($app.DisplayName) has an unexpected role '$roleName' at $scope (not created by this script; review it)."
            }
            elseif (-not $inManaged -and $scope -match '^/subscriptions/[^/]+/?$') {
                Add-Warning "$($app.DisplayName) has a subscription-scope role '$roleName'; least privilege expects resource-group scope only."
            }
        }
    }
}

function Set-WhatIfRoleDefinition {
    param([Parameter(Mandatory)][string]$Scope)

    $o = $script:Run.Options
    $existing = @(Read-AzOrThrow @('role', 'definition', 'list', '--name', $script:WhatIfRoleName, '--custom-role-only', 'true', '--scope', $Scope, '-o', 'json') | Where-Object { $null -ne $_ })
    $label = "custom role $($script:WhatIfRoleName)"
    if ($existing.Count -eq 0) {
        $null = Invoke-Mutation az @('role', 'definition', 'create', '--role-definition', "@$($script:JsonToken)", '-o', 'none') -JsonBody (Get-WhatIfRoleBody -Scope $Scope)
        Add-Item 'RoleAssignments' $label $(if ($o.Apply) { 'Created' } else { 'WouldCreate' }) ($script:WhatIfRoleActions -join ', ')
        return
    }
    $def = $existing[0]
    $perm = @(Get-Prop $def 'permissions' @())
    $actions = @(if ($perm.Count -gt 0) { Get-Prop $perm[0] 'actions' @() })
    $scopes = @(Get-Prop $def 'assignableScopes' @())
    $sameActions = ((@($actions | ForEach-Object { $_.ToLowerInvariant() } | Sort-Object) -join ',') -eq (@($script:WhatIfRoleActions | ForEach-Object { $_.ToLowerInvariant() } | Sort-Object) -join ','))
    $sameScope = @($scopes | Where-Object { $_ -ieq $Scope }).Count -gt 0
    if ($sameActions -and $sameScope) {
        Add-Item 'RoleAssignments' $label 'Exists'
        return
    }
    $null = Invoke-Mutation az @('role', 'definition', 'update', '--role-definition', "@$($script:JsonToken)", '-o', 'none') -JsonBody (Get-WhatIfRoleBody -Scope $Scope -Id ([string](Get-Prop $def 'id' '')))
    Add-Item 'RoleAssignments' $label $(if ($o.Apply) { 'Updated' } else { 'WouldUpdate' })
}

function Step-GrantPromotion {
    $o = $script:Run.Options
    $deploy = @(Get-AppConfigs | Where-Object { $_.Key -eq 'prod' })[0]
    $state = Get-AppState $deploy
    Assert-AppUsable $state
    $dev = (Get-ResourceGroupSpecs)[0]

    $r = Read-Az @('acr', 'list', '--resource-group', $dev.Name, '-o', 'json')
    $registries = @(@(if ($r.Ok) { $r.Data }) | Where-Object { $null -ne $_ })
    if (-not $r.Ok -or $registries.Count -eq 0) {
        Add-Item 'GrantPromotion' 'AcrPull on the dev registry' 'Skipped' "the dev registry does not exist yet in $($dev.Name): deploy dev infrastructure first (relay-deploy), then re-run: bootstrap.ps1 -Step GrantPromotion -Apply"
        return
    }
    if ($registries.Count -gt 1) { throw [System.InvalidOperationException]::new("$($registries.Count) registries in $($dev.Name); expected exactly one.") }
    $registryId = [string](Get-Prop $registries[0] 'id' '')
    $spec = [pscustomobject]@{ Label = "AcrPull for $($deploy.DisplayName) on dev registry $(Get-Prop $registries[0] 'name' '')"; Role = $script:BuiltInRoles.AcrPull; IsCustom = $false; Scope = $registryId; Condition = '' }
    Set-RoleAssignment -AppKey 'prod' -State $state -Spec $spec -Assignments @(Get-RoleAssignmentsOf $state) -StepName 'GrantPromotion'
    $script:Run.Promotion = $registryId
}

# ---- GitHub ----------------------------------------------------------------------------------------------------------

function Get-GitHubVariables {
    $o = $script:Run.Options
    $clientOf = {
        param($key, $placeholder)
        $s = $script:Run.Apps[$key]
        if ($s -and $s.ClientId) { $s.ClientId } else { $placeholder }
    }
    $alert = if ($o.AlertEmail) { $o.AlertEmail } else { '<alert-email>' }
    $dns = $script:Run.DnsSuffix
    $envVars = {
        param($rg, $clientId)
        $v = [ordered]@{}
        $v['AZURE_CLIENT_ID'] = $clientId
        $v['AZURE_TENANT_ID'] = $script:Run.TenantId
        $v['AZURE_SUBSCRIPTION_ID'] = $script:Run.SubscriptionId
        $v['RELAY_RESOURCE_GROUP'] = $rg
        $v['RELAY_DNS_SUFFIX'] = $dns
        $v['RELAY_ALERT_EMAIL'] = $alert
        $v
    }
    $repo = [ordered]@{ AZURE_LOCATION = $o.Location }
    if (-not $o.SkipPrIdentity) {
        $repo['AZURE_PR_CLIENT_ID'] = & $clientOf 'pr' '<pr-client-id>'
        $repo['AZURE_TENANT_ID'] = $script:Run.TenantId
        $repo['AZURE_SUBSCRIPTION_ID'] = $script:Run.SubscriptionId
        $repo['RELAY_PR_RESOURCE_GROUP'] = $o.DevResourceGroup
        $repo['RELAY_DNS_SUFFIX'] = $dns
        $repo['RELAY_ALERT_EMAIL'] = $alert
    }
    [ordered]@{
        Repository   = $repo
        Environments = [ordered]@{
            $o.DevEnvironment  = (& $envVars $o.DevResourceGroup (& $clientOf 'dev' '<dev-client-id>'))
            $o.ProdEnvironment = (& $envVars $o.ProdResourceGroup (& $clientOf 'prod' '<prod-client-id>'))
        }
    }
}

function Get-ReviewerEntries {
    $o = $script:Run.Options
    foreach ($r in $o.ProdReviewers) {
        if ($script:Run.ReviewerIds.ContainsKey($r)) { $script:Run.ReviewerIds[$r] }
        else { [pscustomobject]@{ type = $(if ($r.Contains('/')) { 'Team' } else { 'User' }); id = "<id of $r>" } }
    }
}

# The operations (environment, branch policy, variables) in execution order: shown in dry-run and executed by -ConfigureGitHub.
function Get-GitHubOperations {
    $o = $script:Run.Options
    $repo = $o.Repository
    $vars = Get-GitHubVariables
    $ops = [System.Collections.Generic.List[object]]::new()

    $ops.Add([pscustomobject]@{ Kind = 'Environment'; Name = $o.DevEnvironment; Target = $o.DevEnvironment; Args = @('api', '-X', 'PUT', "repos/$repo/environments/$($o.DevEnvironment)"); Json = '' })
    $body = [ordered]@{
        reviewers              = @(Get-ReviewerEntries | ForEach-Object { [ordered]@{ type = $_.type; id = $_.id } })
        deployment_branch_policy = [ordered]@{ protected_branches = $false; custom_branch_policies = $true }
    } | ConvertTo-Json -Compress -Depth 5
    $ops.Add([pscustomobject]@{ Kind = 'Environment'; Name = $o.ProdEnvironment; Target = $o.ProdEnvironment; Args = @('api', '-X', 'PUT', "repos/$repo/environments/$($o.ProdEnvironment)", '--input', $script:JsonToken); Json = $body })
    $ops.Add([pscustomobject]@{ Kind = 'BranchPolicy'; Name = "$($o.ProdEnvironment) deploys only from $($o.DeploymentBranch)"; Target = $o.ProdEnvironment; Args = @('api', '-X', 'POST', "repos/$repo/environments/$($o.ProdEnvironment)/deployment-branch-policies", '-f', "name=$($o.DeploymentBranch)", '-f', 'type=branch'); Json = '' })
    foreach ($k in $vars.Repository.Keys) {
        $ops.Add([pscustomobject]@{ Kind = 'Variable'; Name = $k; Target = ''; Value = [string]$vars.Repository[$k]; Args = @('variable', 'set', $k, '--repo', $repo, '--body', [string]$vars.Repository[$k]); Json = '' })
    }
    foreach ($envName in $vars.Environments.Keys) {
        foreach ($k in $vars.Environments[$envName].Keys) {
            $ops.Add([pscustomobject]@{ Kind = 'Variable'; Name = $k; Target = $envName; Value = [string]$vars.Environments[$envName][$k]; Args = @('variable', 'set', $k, '--env', $envName, '--repo', $repo, '--body', [string]$vars.Environments[$envName][$k]); Json = '' })
        }
    }
    $ops.ToArray()
}

function Resolve-Reviewers {
    foreach ($r in $script:Run.Options.ProdReviewers) {
        $isTeam = $r.Contains('/')
        $path = if ($isTeam) { $parts = $r -split '/', 2; "orgs/$($parts[0])/teams/$($parts[1])" } else { "users/$r" }
        $res = Read-Gh @('api', $path, '--jq', '.id')
        if (-not $res.Ok -or $null -eq $res.Data) { throw [System.InvalidOperationException]::new("Could not resolve GitHub reviewer '$r': $($res.Error)") }
        $script:Run.ReviewerIds[$r] = [pscustomobject]@{ type = $(if ($isTeam) { 'Team' } else { 'User' }); id = [int64]$res.Data }
    }
}

function Step-GitHub {
    $o = $script:Run.Options
    if (-not $o.ConfigureGitHub) {
        Add-Item 'GitHub' 'environments and variables' 'Skipped' 'not configured: the commands are printed below (use -ConfigureGitHub to run them)'
        return
    }
    Resolve-Reviewers
    $repo = $o.Repository
    $apply = $o.Apply

    $devExists = (Read-Gh @('api', "repos/$repo/environments/$($o.DevEnvironment)")).Ok
    $prodRes = Read-Gh @('api', "repos/$repo/environments/$($o.ProdEnvironment)")
    $ops = @(Get-GitHubOperations)

    foreach ($op in $ops | Where-Object { $_.Kind -eq 'Environment' }) {
        $isProd = $op.Name -eq $o.ProdEnvironment
        $state = if ($isProd) { $prodRes } else { [pscustomobject]@{ Ok = $devExists; Data = $null } }
        if (-not $state.Ok) {
            $null = Invoke-Mutation gh $op.Args -JsonBody $op.Json
            Add-Item 'GitHub' "environment $($op.Name)" $(if ($apply) { 'Created' } else { 'WouldCreate' })
            continue
        }
        if (-not $isProd) {
            $rules = @(Get-Prop $state.Data 'protection_rules' @() | Where-Object { (Get-Prop $_ 'type' '') -eq 'required_reviewers' })
            if ($rules.Count -gt 0) { Add-Warning "Environment $($op.Name) has required reviewers; dev should deploy without approval." }
            Add-Item 'GitHub' "environment $($op.Name)" 'Exists'
            continue
        }
        $rules = @(Get-Prop $state.Data 'protection_rules' @() | Where-Object { (Get-Prop $_ 'type' '') -eq 'required_reviewers' })
        $haveIds = @($rules | ForEach-Object { Get-Prop $_ 'reviewers' @() } | ForEach-Object { [string](Get-Prop (Get-Prop $_ 'reviewer' $null) 'id' '') } | Sort-Object -Unique)
        $wantIds = @(Get-ReviewerEntries | ForEach-Object { [string]$_.id } | Sort-Object -Unique)
        $policy = Get-Prop $state.Data 'deployment_branch_policy' $null
        $custom = $null -ne $policy -and (Get-Prop $policy 'custom_branch_policies' $false) -eq $true
        if (($haveIds -join ',') -eq ($wantIds -join ',') -and $custom) {
            Add-Item 'GitHub' "environment $($op.Name)" 'Exists'
        }
        else {
            $null = Invoke-Mutation gh $op.Args -JsonBody $op.Json
            Add-Item 'GitHub' "environment $($op.Name)" $(if ($apply) { 'Updated' } else { 'WouldUpdate' }) 'reviewers/branch restriction'
        }
    }

    $policies = Read-Gh @('api', "repos/$repo/environments/$($o.ProdEnvironment)/deployment-branch-policies")
    $haveBranch = $policies.Ok -and $null -ne $policies.Data -and @(Get-Prop $policies.Data 'branch_policies' @() | Where-Object { (Get-Prop $_ 'name' '') -eq $o.DeploymentBranch }).Count -gt 0
    $branchOp = $ops | Where-Object { $_.Kind -eq 'BranchPolicy' }
    if ($haveBranch) { Add-Item 'GitHub' $branchOp.Name 'Exists' }
    else {
        $null = Invoke-Mutation gh $branchOp.Args
        Add-Item 'GitHub' $branchOp.Name $(if ($apply) { 'Created' } else { 'WouldCreate' })
    }

    $existingVars = @{}
    foreach ($scopeKey in @('') + @($o.DevEnvironment, $o.ProdEnvironment)) {
        $listArgs = @('variable', 'list', '--repo', $repo, '--json', 'name,value')
        if ($scopeKey) { $listArgs += @('--env', $scopeKey) }
        $res = Read-Gh $listArgs
        $map = @{}
        if ($res.Ok -and $null -ne $res.Data) { foreach ($v in @($res.Data)) { $map[[string](Get-Prop $v 'name' '')] = [string](Get-Prop $v 'value' '') } }
        $existingVars[$scopeKey] = $map
    }
    foreach ($op in $ops | Where-Object { $_.Kind -eq 'Variable' }) {
        $label = if ($op.Target) { "$($op.Target): $($op.Name)" } else { "repository: $($op.Name)" }
        $map = $existingVars[$op.Target]
        if ($map.ContainsKey($op.Name) -and $map[$op.Name] -ceq $op.Value) { Add-Item 'GitHub' $label 'Exists'; continue }
        $had = $map.ContainsKey($op.Name)
        $null = Invoke-Mutation gh $op.Args
        Add-Item 'GitHub' $label $(if ($apply) { if ($had) { 'Updated' } else { 'Created' } } else { if ($had) { 'WouldUpdate' } else { 'WouldCreate' } })
    }
}

# ---- steps: teardown -------------------------------------------------------------------------------------------------

function Remove-RoleAssignments {
    $o = $script:Run.Options
    $rgs = @(Get-ResourceGroupSpecs)
    foreach ($app in Get-AppConfigs) {
        $state = Get-AppState $app
        if (-not $state.SpExists) { Add-Item 'RoleAssignments' $app.DisplayName 'Absent' 'no service principal'; continue }
        $mine = @(Get-RoleAssignmentsOf $state | Where-Object {
                $scope = [string](Get-Prop $_ 'scope' '')
                @($rgs | Where-Object { $scope -ieq $_.Scope -or $scope.StartsWith($_.Scope + '/', [System.StringComparison]::OrdinalIgnoreCase) }).Count -gt 0
            })
        if ($mine.Count -eq 0) { Add-Item 'RoleAssignments' $app.DisplayName 'Absent' 'no role assignments in the managed resource groups'; continue }
        foreach ($a in $mine) {
            $null = Invoke-Mutation az @('role', 'assignment', 'delete', '--ids', [string](Get-Prop $a 'id' ''), '-o', 'none')
            Add-Item 'RoleAssignments' "$($app.DisplayName): $(Get-Prop $a 'roleDefinitionName' 'role') at $(Get-Prop $a 'scope' '')" $(if ($o.Apply) { 'Removed' } else { 'WouldRemove' })
        }
    }
    if (-not $o.SkipPrIdentity) {
        $dev = (Get-ResourceGroupSpecs)[0]
        $defs = @(Read-AzOrThrow @('role', 'definition', 'list', '--name', $script:WhatIfRoleName, '--custom-role-only', 'true', '--scope', $dev.Scope, '-o', 'json') | Where-Object { $null -ne $_ })
        if ($defs.Count -gt 0) {
            $null = Invoke-Mutation az @('role', 'definition', 'delete', '--name', $script:WhatIfRoleName, '--scope', $dev.Scope, '-o', 'none')
            Add-Item 'RoleAssignments' "custom role $($script:WhatIfRoleName)" $(if ($o.Apply) { 'Removed' } else { 'WouldRemove' })
        }
        else { Add-Item 'RoleAssignments' "custom role $($script:WhatIfRoleName)" 'Absent' }
    }
}

function Remove-FederatedCredentials {
    $o = $script:Run.Options
    foreach ($app in Get-AppConfigs) {
        $state = Get-AppState $app
        if (-not $state.Exists) { Add-Item 'FederatedCredentials' $app.DisplayName 'Absent' 'no app registration'; continue }
        $existing = @(Read-AzOrThrow @('ad', 'app', 'federated-credential', 'list', '--id', $state.ClientId, '-o', 'json') | Where-Object { $null -ne $_ })
        foreach ($spec in $app.Credentials) {
            $found = @($existing | Where-Object { (Get-Prop $_ 'name' '') -ceq $spec.Name -or (Get-Prop $_ 'subject' '') -ceq $spec.Subject })
            if ($found.Count -eq 0) { Add-Item 'FederatedCredentials' "$($app.DisplayName): $($spec.Name)" 'Absent'; continue }
            $null = Invoke-Mutation az @('ad', 'app', 'federated-credential', 'delete', '--id', $state.ClientId, '--federated-credential-id', [string](Get-Prop $found[0] 'name' $spec.Name), '-o', 'none')
            Add-Item 'FederatedCredentials' "$($app.DisplayName): $($spec.Name)" $(if ($o.Apply) { 'Removed' } else { 'WouldRemove' })
        }
    }
}

function Remove-Identity {
    $o = $script:Run.Options
    foreach ($app in Get-AppConfigs) {
        $state = Get-AppState $app
        if (-not $state.Exists) { Add-Item 'Identity' "app $($app.DisplayName)" 'Absent'; continue }
        $null = Invoke-Mutation az @('ad', 'app', 'delete', '--id', $state.ClientId)
        Add-Item 'Identity' "app $($app.DisplayName) (and its service principal)" $(if ($o.Apply) { 'Removed' } else { 'WouldRemove' })
    }
}

function Remove-ResourceGroups {
    $o = $script:Run.Options
    if (-not $o.RemoveResourceGroups) {
        Add-Item 'ResourceGroups' 'resource groups' 'Skipped' 'kept (add -RemoveResourceGroups to delete them)'
        return
    }
    foreach ($rg in Get-ResourceGroupSpecs) {
        $exists = Read-AzOrThrow @('group', 'exists', '--name', $rg.Name)
        if ($exists -ne $true) { Add-Item 'ResourceGroups' $rg.Name 'Absent'; continue }
        $current = Read-AzOrThrow @('group', 'show', '--name', $rg.Name, '-o', 'json')
        $tags = Get-Prop $current 'tags' $null
        $managed = $null -ne $tags -and (Get-Prop $tags 'managedBy' '') -eq 'bootstrap'
        if (-not $managed) { Add-Item 'ResourceGroups' $rg.Name 'Skipped' 'not tagged managedBy=bootstrap: delete it yourself if intended'; continue }
        $null = Invoke-Mutation az @('group', 'delete', '--name', $rg.Name, '--yes', '-o', 'none')
        Add-Item 'ResourceGroups' $rg.Name $(if ($o.Apply) { 'Removed' } else { 'WouldRemove' })
    }
}

# ---- preconditions ----------------------------------------------------------------------------------------------------

function Add-Precondition {
    param([string]$Id, [string]$Status, [string]$Message)
    $script:Run.Preconditions.Add([pscustomobject]@{ Id = $Id; Status = $Status; Message = $Message })
    $color = @{ PASS = 'Green'; WARN = 'Yellow'; FAIL = 'Red' }[$Status]
    Write-Log ("[{0}] {1}: {2}" -f $Status, $Id, $Message) $color
}

function Get-PrincipalObjectId {
    param($Account)
    $user = Get-Prop $Account 'user' $null
    if ($null -eq $user) { return '' }
    $name = [string](Get-Prop $user 'name' '')
    if ((Get-Prop $user 'type' '') -eq 'servicePrincipal') {
        $r = Read-Az @('ad', 'sp', 'show', '--id', $name, '--query', 'id', '-o', 'json')
    }
    else {
        $r = Read-Az @('ad', 'signed-in-user', 'show', '--query', 'id', '-o', 'json')
    }
    if ($r.Ok -and $r.Data) { return [string]$r.Data }
    ''
}

function Test-BootstrapPermissions {
    param($Account)

    $sub = $script:Run.SubscriptionId
    $ok = $true
    $oid = Get-PrincipalObjectId $Account
    if (-not $oid) {
        Add-Precondition 'azure-role' 'WARN' "Could not read the signed-in principal's object id, so the Azure role was not verified. Verify manually: you need Owner (or User Access Administrator + Contributor) on subscription $sub."
    }
    else {
        $r = Read-Az @('role', 'assignment', 'list', '--assignee', $oid, '--all', '--include-inherited', '--include-groups', '-o', 'json')
        if (-not $r.Ok) {
            Add-Precondition 'azure-role' 'WARN' "Could not list your role assignments ($($r.Error)). Verify manually: Owner (or User Access Administrator + Contributor) on subscription $sub."
        }
        else {
            $names = @(@($r.Data) | Where-Object {
                    $null -ne $_ -and ([string](Get-Prop $_ 'scope' '') -match "(?i)^(/|/subscriptions/$([regex]::Escape($sub))/?|/providers/Microsoft\.Management/.*)$")
                } | ForEach-Object { [string](Get-Prop $_ 'roleDefinitionName' '') })
            $canAssign = ($names -contains 'Owner') -or ((($names -contains 'User Access Administrator') -or ($names -contains 'Role Based Access Control Administrator')) -and ($names -contains 'Contributor'))
            if ($canAssign) { Add-Precondition 'azure-role' 'PASS' 'You can create resource groups and role assignments (Owner, or UAA/RBAC Administrator with Contributor).' }
            else { Add-Precondition 'azure-role' 'FAIL' "None of Owner, or User Access Administrator + Contributor was found on subscription $sub (found: $($names -join ', ')). Ask an owner, or pass -SkipPermissionCheck if the access comes from somewhere this check cannot see (PIM, resource-group scope)."; $ok = $false }
        }
    }

    $roles = Read-Az @('rest', '--method', 'GET', '--url', 'https://graph.microsoft.com/v1.0/me/transitiveMemberOf/microsoft.graph.directoryRole?$select=displayName', '--query', 'value[].displayName', '-o', 'json')
    $entraRoles = @(if ($roles.Ok -and $null -ne $roles.Data) { @($roles.Data) })
    $appRoles = 'Application Administrator', 'Cloud Application Administrator', 'Global Administrator', 'Application Developer'
    if (@($entraRoles | Where-Object { $appRoles -contains $_ }).Count -gt 0) {
        Add-Precondition 'entra' 'PASS' 'You hold an Entra role that can register applications.'
        return $ok
    }
    $policy = Read-Az @('rest', '--method', 'GET', '--url', 'https://graph.microsoft.com/v1.0/policies/authorizationPolicy', '--query', 'defaultUserRolePermissions.allowedToCreateApps', '-o', 'json')
    if ($policy.Ok -and $policy.Data -eq $true) { Add-Precondition 'entra' 'PASS' 'The tenant lets users register applications.' }
    elseif ($policy.Ok -and $policy.Data -eq $false) { Add-Precondition 'entra' 'FAIL' 'The tenant does not let users register applications and you hold no Application Administrator role. Ask a tenant admin for Application Administrator (or Cloud Application Administrator),     or pass -SkipPermissionCheck.'; $ok = $false }
    else { Add-Precondition 'entra' 'WARN' 'Could not determine whether you may register applications. Verify manually: Entra roles Application Administrator / Cloud Application Administrator, or "Users can register applications" = Yes. The Identity step fails clearly if not.' }
        $ok
    }

function Test-Preconditions {
    $o = $script:Run.Options

    if ($null -eq $script:AzRunner -and -not (Get-Command az -ErrorAction SilentlyContinue)) {
        Add-Precondition 'az' 'FAIL' 'The Azure CLI (az) was not found on PATH. Install it (https://aka.ms/installazurecli) and run az login.'
        return $false
    }
    if ($o.ConfigureGitHub -and $null -eq $script:GhRunner -and -not (Get-Command gh -ErrorAction SilentlyContinue)) {
        Add-Precondition 'gh' 'FAIL' '-ConfigureGitHub needs the GitHub CLI (gh) on PATH. Install it and run gh auth login, or omit -ConfigureGitHub to print the commands.'
        return $false
    }

    try {
        $acct = Read-Az @('account', 'show', '-o', 'json')
        if (-not $acct.Ok -or $null -eq $acct.Data) { throw [System.InvalidOperationException]::new("AZ_AUTH: $($acct.Error)") }
    }
    catch [System.InvalidOperationException] {
        if ($_.Exception.Message -notlike 'AZ_AUTH:*') { throw }
        Add-Precondition 'login' 'FAIL' "Not logged in to Azure or the session expired. Run az login yourself (this script never logs in). $($_.Exception.Message.Substring(8).Trim())"
        return $false
    }
    $script:Run.SubscriptionId = [string](Get-Prop $acct.Data 'id' '')
    $script:Run.TenantId = [string](Get-Prop $acct.Data 'tenantId' '')
    $script:Run.SubscriptionName = [string](Get-Prop $acct.Data 'name' '')
    $script:Run.Account = $acct.Data
    if (-not $script:Run.SubscriptionId) { Add-Precondition 'account' 'FAIL' 'az account show returned no subscription id.'; return $false }
    if ((Get-Prop $acct.Data 'state' 'Enabled') -ne 'Enabled') { Add-Precondition 'account' 'FAIL' "Subscription '$($script:Run.SubscriptionName)' is not Enabled."; return $false }
    Add-Precondition 'account' 'PASS' "Subscription '$($script:Run.SubscriptionName)' ($($script:Run.SubscriptionId)), tenant $($script:Run.TenantId)."

    $ok = $true
    if (-not $o.SkipPreflight -and -not $o.Remove) {
        $pfResult = Invoke-Preflight -Location $o.Location -SubscriptionId $o.SubscriptionId
        if ($pfResult.ExitCode -ne 0) {
            foreach ($c in @($pfResult.Checks | Where-Object { $_.Status -eq 'FAIL' })) {
                Add-Precondition "preflight:$($c.Id)" 'FAIL' "$($c.Message) $($c.Remediation)".Trim()
            }
            Add-Precondition 'preflight' 'FAIL' 'The preflight checks failed (run infra/relay/preflight.ps1 for the full report, or -SkipPreflight).'
            $ok = $false
        }
        else {
            $warns = @($pfResult.Checks | Where-Object { $_.Status -eq 'WARN' }).Count
            Add-Precondition 'preflight' 'PASS' "Preflight checks passed ($warns warning(s))."
        }
    }
    if (-not $ok) { return $false }

    if (-not $o.SkipPermissionCheck -and -not (Test-BootstrapPermissions $acct.Data)) { return $false }

    if ($o.ConfigureGitHub) {
        $gh = Invoke-Tool -Tool gh -ToolArgs @('auth', 'status')
        if ($gh.ExitCode -ne 0) {
            Add-Precondition 'gh-auth' 'FAIL' '-ConfigureGitHub refused: gh is not authenticated. Run gh auth login (repository admin), then retry, or omit -ConfigureGitHub to print the commands.'
            return $false
        }
        Add-Precondition 'gh-auth' 'PASS' 'gh is authenticated.'
        if ([string]::IsNullOrWhiteSpace($o.AlertEmail)) { Add-Precondition 'alert-email' 'FAIL' '-ConfigureGitHub needs -AlertEmail (published as RELAY_ALERT_EMAIL).'; $ok = $false }
        if (@($o.ProdReviewers).Count -eq 0 -and $o.Step -match 'All|GitHub') { Add-Precondition 'prod-reviewers' 'FAIL' "-ConfigureGitHub needs -ProdReviewers (GitHub logins or org/team slugs): $($o.ProdEnvironment) must require approval."; $ok = $false }
    }
    $ok
}

function Confirm-Apply {
    $o = $script:Run.Options
    $what = if ($o.Remove) { 'DELETE' } else { 'CREATE/MODIFY' }
    $prompt = "About to $what Azure/Entra objects in subscription '$($script:Run.SubscriptionName)' ($(Format-MaskedId $script:Run.SubscriptionId)): steps $((Get-SelectedSteps) -join ', ')$(if ($o.RemoveResourceGroups) { ' INCLUDING the resource groups' }). Type 'yes' to continue"
    if ($null -ne $script:ConfirmRunner) { return [bool](& $script:ConfirmRunner $prompt) }
    if ([Console]::IsInputRedirected) {
        Write-Log 'Non-interactive session: pass -Force to confirm -Apply.' 'Red'
        return $false
    }
    (Read-Host $prompt) -eq 'yes'
}

# ---- orchestration ----------------------------------------------------------------------------------------------------

function Invoke-Bootstrap {
    [CmdletBinding()]
    param([hashtable]$Options = @{})

    $script:Run = [pscustomobject]@{
        Options = $null; SubscriptionId = ''; TenantId = ''; SubscriptionName = ''; Account = $null; DnsSuffix = ''
        Items = [System.Collections.Generic.List[object]]::new()
        Warnings = [System.Collections.Generic.List[string]]::new()
        Planned = [System.Collections.Generic.List[object]]::new()
        Executed = [System.Collections.Generic.List[object]]::new()
        Preconditions = [System.Collections.Generic.List[object]]::new()
        Apps = @{}; ReviewerIds = @{}; Promotion = ''
        Failed = [System.Collections.Generic.HashSet[string]]::new()
        ExitCode = 0
    }
    $finish = {
        param([int]$Code)
        $script:Run.ExitCode = $Code
        $script:Run
    }

    try { $script:Run.Options = Resolve-BootstrapOptions $Options }
    catch [System.ArgumentException] {
        $script:Run.Options = Resolve-BootstrapOptions @{}
        $script:Run.Preconditions.Add([pscustomobject]@{ Id = 'usage'; Status = 'FAIL'; Message = $_.Exception.Message })
        return (& $finish 2)
    }
    $o = $script:Run.Options
    Write-Log ("WPI Emulator relay bootstrap: {0}{1}" -f $(if ($o.Remove) { 'REMOVE' } else { 'setup' }), $(if ($o.Apply) { ' (APPLY)' } else { ' (dry run: nothing is changed; add -Apply)' })) 'Cyan'

    $pre = $false
    try { $pre = Test-Preconditions }
    catch [System.InvalidOperationException] {
        Add-Precondition 'precondition' 'FAIL' $_.Exception.Message
    }
    if (-not $pre) { return (& $finish 2) }

    $script:Run.DnsSuffix = if ($o.DnsSuffix) { $o.DnsSuffix } else { Get-DefaultDnsSuffix $script:Run.SubscriptionId }
    try { foreach ($app in Get-AppConfigs) { $null = Get-AppState $app } }
    catch [System.InvalidOperationException] {
        Add-Precondition 'app-registrations' 'FAIL' $_.Exception.Message
        return (& $finish 2)
    }
    if ($o.Apply -and -not $o.Force -and -not (Confirm-Apply)) {
        Add-Precondition 'confirmation' 'FAIL' 'Not confirmed; nothing was changed.'
        return (& $finish 2)
    }

    $selected = @(Get-SelectedSteps)
    $order = if ($o.Remove) { @('GitHub', 'GrantPromotion', 'RoleAssignments', 'FederatedCredentials', 'Identity', 'ResourceGroups') } else { $script:StepOrder }
    $removeDeps = @{ RoleAssignments = @(); FederatedCredentials = @('RoleAssignments'); Identity = @('FederatedCredentials'); ResourceGroups = @('Identity') }

    foreach ($name in $order) {
        if ($selected -notcontains $name) { continue }
        Write-Log "== $name" 'Cyan'
        if ($o.Remove -and $name -in 'GitHub', 'GrantPromotion') {
            if ($name -eq 'GitHub') { Add-Item $name 'environments and variables' 'Skipped' 'not removed: delete them in the repository settings' }
            continue
        }
        $deps = if ($o.Remove) { @($removeDeps[$name]) } else { @($script:StepDependencies[$name]) }
        $blocked = @($deps | Where-Object { $script:Run.Failed.Contains($_) })
        if ($blocked.Count -gt 0) {
            Add-Item $name '-' 'Skipped' "depends on failed step $($blocked -join ', ')"
            $null = $script:Run.Failed.Add($name)
            continue
        }
        try {
            if ($o.Remove) {
                switch ($name) {
                    'RoleAssignments' { Remove-RoleAssignments }
                    'FederatedCredentials' { Remove-FederatedCredentials }
                    'Identity' { Remove-Identity }
                    'ResourceGroups' { Remove-ResourceGroups }
                }
            }
            else {
                switch ($name) {
                    'ResourceGroups' { Step-ResourceGroups }
                    'Identity' { Step-Identity }
                    'FederatedCredentials' { Step-FederatedCredentials }
                    'RoleAssignments' { Step-RoleAssignments }
                    'GrantPromotion' { Step-GrantPromotion }
                    'GitHub' { Step-GitHub }
                }
            }
        }
        catch {
            Add-Item $name '-' 'Failed' $_.Exception.Message
            $null = $script:Run.Failed.Add($name)
        }
    }
    & $finish $(if ($script:Run.Failed.Count -gt 0) { 1 } else { 0 })
}

function Get-FinalReport {
    $o = $script:Run.Options
    $r = $script:Run
    $github = $null
    if (-not $o.Remove -and $r.ExitCode -ne 2) {
        $ops = @(Get-GitHubOperations)
        $vars = Get-GitHubVariables
        $github = [ordered]@{
            Repository   = $o.Repository
            Environments = @(
                [ordered]@{ Name = $o.DevEnvironment; RequiredReviewers = $false; DeploymentBranches = 'any' },
                [ordered]@{ Name = $o.ProdEnvironment; RequiredReviewers = @($o.ProdReviewers); DeploymentBranches = $o.DeploymentBranch }
            )
            Variables    = $vars
            Commands     = @($ops | ForEach-Object { Protect-Text (Format-Command -Tool gh -ToolArgs $_.Args -JsonBody $_.Json) } | ForEach-Object { $_ })
            CommandsFull = @($ops | ForEach-Object { Format-Command -Tool gh -ToolArgs $_.Args -JsonBody $_.Json })
        }
    }
    $next = [System.Collections.Generic.List[string]]::new()
    if ($r.ExitCode -ne 2 -and -not $o.Apply) { $next.Add('This was a dry run. Review the commands above, then re-run with -Apply (add -Force to skip the confirmation).') }
    if (-not $o.Remove -and $r.ExitCode -ne 2) {
        if (-not $o.ConfigureGitHub) { $next.Add('Configure GitHub with the commands below, or re-run with -ConfigureGitHub -ProdReviewers <login> -AlertEmail <email> (needs gh auth login).') }
        $promo = @($r.Items | Where-Object { $_.Step -eq 'GrantPromotion' -and $_.Status -eq 'Skipped' }).Count
        if ($promo -gt 0) { $next.Add('After the first dev infrastructure deployment creates the dev registry, run: pwsh infra/relay/bootstrap.ps1 -Step GrantPromotion -Apply') }
        $next.Add('Federated credentials can take a minute to propagate: if the first workflow login fails with AADSTS70025/AADSTS700213, wait and re-run it.')
    }
    [pscustomobject]@{
        Mode           = if ($o.Apply) { 'Apply' } else { 'DryRun' }
        Operation      = if ($o.Remove) { 'Remove' } else { 'Setup' }
        ExitCode       = $r.ExitCode
        SubscriptionId = $r.SubscriptionId
        TenantId       = $r.TenantId
        DnsSuffix      = $r.DnsSuffix
        Preconditions  = @($r.Preconditions)
        Steps          = @($r.Items)
        Apps           = @(Get-AppConfigs | ForEach-Object {
                $s = $r.Apps[$_.Key]
                [ordered]@{
                    Key = $_.Key; DisplayName = $_.DisplayName
                    ClientId = $(if ($s -and $s.ClientId) { $s.ClientId } else { '' })
                    ServicePrincipalObjectId = $(if ($s -and $s.SpObjectId) { $s.SpObjectId } else { '' })
                    FederatedCredentialSubjects = @($_.Credentials | ForEach-Object { $_.Subject })
                    RoleScope = (Get-AppHomeScope $_.Key)
                }
            })
        Warnings       = @($r.Warnings)
        PlannedCommands = @($r.Planned | ForEach-Object { $_.Command })
        GitHub         = $github
        NextSteps      = $next.ToArray()
    }
}

function Write-BootstrapReport {
    param($Report)

    Write-Host ''
    Write-Host "Summary ($($Report.Mode), $($Report.Operation))" -ForegroundColor Cyan
    foreach ($i in $Report.Steps) { Write-Host (Protect-Text ('  {0,-12} {1,-22} {2}' -f $i.Status, $i.Step, $i.Item)) }
    if ($Report.Apps.Count -gt 0) {
        Write-Host 'App registrations (client ids are identifiers, not secrets)' -ForegroundColor Cyan
        foreach ($a in $Report.Apps) { Write-Host (Protect-Text ('  {0,-26} client id {1}  roles only under {2}' -f $a.DisplayName, $a.ClientId, $a.RoleScope)) }
    }
    if ($Report.Warnings.Count -gt 0) { Write-Host 'Warnings:' -ForegroundColor Yellow; foreach ($w in $Report.Warnings) { Write-Host (Protect-Text "  $w") } }
    if ($Report.GitHub) {
        Write-Host ''
        Write-Host 'GitHub configuration (identifiers, not secrets; copy exactly)' -ForegroundColor Cyan
        Write-Host "  Repository: $($Report.GitHub.Repository)"
        Write-Host "  Environment $($Report.GitHub.Environments[0].Name): no required reviewers"
        Write-Host "  Environment $($Report.GitHub.Environments[1].Name): required reviewers $((@($Report.GitHub.Environments[1].RequiredReviewers) -join ', ')); deployments only from '$($Report.GitHub.Environments[1].DeploymentBranches)'"
        Write-Host '  Repository variables:'
        foreach ($k in $Report.GitHub.Variables.Repository.Keys) { Write-Host "    $k = $($Report.GitHub.Variables.Repository[$k])" }
        foreach ($e in $Report.GitHub.Variables.Environments.Keys) {
            Write-Host "  Variables of environment ${e}:"
            foreach ($k in $Report.GitHub.Variables.Environments[$e].Keys) { Write-Host "    $k = $($Report.GitHub.Variables.Environments[$e][$k])" }
        }
        Write-Host '  Commands (gh auth login first; PowerShell syntax):'
        foreach ($c in $Report.GitHub.CommandsFull) { Write-Host "    $c" }
        Write-Host '  Reviewer ids: gh api users/<login> --jq .id  (teams: gh api orgs/<org>/teams/<slug> --jq .id)'
    }
    foreach ($n in $Report.NextSteps) { Write-Host "Next: $n" -ForegroundColor Yellow }
    Write-Host ("Result: exit {0}" -f $Report.ExitCode)
}

if (-not $script:DotSourced) {
    $run = Invoke-Bootstrap -Options $script:CliOptions
    $report = Get-FinalReport
    if ($run.Options.Json) { $report | ConvertTo-Json -Depth 8 }
    else { Write-BootstrapReport -Report $report }
    exit $run.ExitCode
}
