// WPI Emulator practice-match relay: infrastructure of ONE environment (dev or prod) in ONE resource group.
// Deploy dev and prod to separate resource groups with their own parameter file, for example:
//   az deployment group create --resource-group rg-wpiemu-relay-dev --template-file main.bicep --parameters main.dev.bicepparam
// First pass: deployRelay=false creates the registry, identity, logs and budget, so the image can be pushed.
// Second pass: deployRelay=true with relayImage=<registry>/wpiemu-relay@sha256:<digest> creates the container group.

targetScope = 'resourceGroup'

@description('Azure region of every resource. South Central US was verified with infra/relay/preflight.ps1.')
param location string = 'southcentralus'

@description('Environment this deployment belongs to. It is part of the resource names and DNS label.')
@allowed([
  'dev'
  'prod'
])
param environmentName string

@description('Prefix of every resource name and of the DNS label.')
@minLength(3)
@maxLength(24)
param namePrefix string = 'wpiemu-relay'

@description('''Suffix that makes the DNS label unique: ACI labels are global per region. The label is <namePrefix><suffix> in prod and
<namePrefix>-dev<suffix> in dev (for example "47c926" or "-a1b2"). Use lower-case letters, digits and hyphens, and do not end with a
hyphen. The apps build the relay address from the same suffix (app/ui/relay_endpoints.h).''')
@minLength(1)
@maxLength(30)
param dnsLabelSuffix string

@description('''Relay image reference by digest, for example myregistry.azurecr.io/wpiemu-relay@sha256:<digest>. Leave it empty
together with deployRelay=false for the first pass, which creates the registry before an image exists.''')
param relayImage string = ''

@description('Create the container group. False creates only the registry, identity, logs and budget.')
param deployRelay bool = true

@description('''UDP port of the relay. 7777 is the default. UDP 443 is often open on school and corporate networks that block other
UDP ports (v1 has no TCP fallback), but ports below 1024 need the relay to run as root inside the container (ACI cannot add
capabilities), so pick 443 only after the image has been checked for that. Changing the port changes the public endpoint.''')
@minValue(1)
@maxValue(65535)
param udpPort int = 7777

@description('vCPUs of the relay container, a decimal string.')
@allowed([
  '0.5'
  '1'
  '2'
])
param cpu string = '0.5'

@description('Memory of the relay container in GB, a decimal string.')
@allowed([
  '0.5'
  '1.0'
  '2.0'
])
param memoryGb string = '1.0'

@description('Rooms the relay hosts at once (WPIEMU_RELAY_MAX_ROOMS, relay limit 500).')
@minValue(1)
@maxValue(500)
param maxRooms int = 100

@description('Milliseconds a room survives without its host (WPIEMU_RELAY_ROOM_TTL_MS).')
@minValue(1000)
@maxValue(86400000)
param roomTtlMs int = 30000

@description('Token-bucket rate per client in bytes per second (WPIEMU_RELAY_PEER_BYTES_PER_SEC).')
@minValue(1024)
@maxValue(100000000)
param peerBytesPerSec int = 65536

@description('Forwarded bytes per second per room (WPIEMU_RELAY_ROOM_BYTES_PER_SEC); 0 derives it from the peer rate.')
@minValue(0)
@maxValue(1000000000)
param roomBytesPerSec int = 0

@description('Days the relay logs are kept in Log Analytics.')
@minValue(30)
@maxValue(730)
param logRetentionDays int = 30

@description('Daily Log Analytics ingestion cap in GB, a decimal string. Ingestion stops for the day once it is reached.')
param dailyLogCapGb string = '0.5'

@description('Email for the budget alerts (80% actual, 100% forecast). Empty: no budget is created, because a budget without a notification has no effect.')
param alertEmail string = ''

@description('Monthly budget in USD for the resource group.')
@minValue(1)
param monthlyBudgetUsd int = 30

@description('First day of the budget month (yyyy-MM-01). Defaults to the current month.')
param budgetStartDate string = utcNow('yyyy-MM-01')

@description('Extra tags applied to every resource.')
param tags object = {}

var resourceTags = union(tags, {
  application: 'wpiemu-relay'
  environment: environmentName
  managedBy: 'bicep'
})

var nameSuffix = '${namePrefix}-${environmentName}'
var registryName = '${replace(namePrefix, '-', '')}${environmentName}${uniqueString(resourceGroup().id)}'
var dnsNameLabel = environmentName == 'prod' ? '${namePrefix}${dnsLabelSuffix}' : '${namePrefix}-${environmentName}${dnsLabelSuffix}'

// Names and defaults are the relay's own (relay/src/relay_options.cpp); infra/relay/tests/bicep.tests.ps1 checks them.
var relayEnvironment = [
  { name: 'WPIEMU_RELAY_PORT', value: string(udpPort) }
  { name: 'WPIEMU_RELAY_MAX_ROOMS', value: string(maxRooms) }
  { name: 'WPIEMU_RELAY_ROOM_TTL_MS', value: string(roomTtlMs) }
  { name: 'WPIEMU_RELAY_PEER_BYTES_PER_SEC', value: string(peerBytesPerSec) }
  { name: 'WPIEMU_RELAY_ROOM_BYTES_PER_SEC', value: string(roomBytesPerSec) }
  { name: 'WPIEMU_RELAY_LOG', value: 'json' }
]

module registry 'modules/registry.bicep' = {
  name: 'registry'
  params: {
    registryName: registryName
    location: location
    tags: resourceTags
  }
}

module identity 'modules/identity.bicep' = {
  name: 'identity'
  params: {
    identityName: 'id-${nameSuffix}'
    location: location
    registryName: registry.outputs.registryName
    tags: resourceTags
  }
}

module logs 'modules/logs.bicep' = {
  name: 'logs'
  params: {
    workspaceName: 'log-${nameSuffix}'
    location: location
    retentionInDays: logRetentionDays
    dailyQuotaGb: dailyLogCapGb
    tags: resourceTags
  }
}

// The container group starts only after the identity module, which includes the AcrPull assignment.
module relay 'modules/relay.bicep' = if (deployRelay) {
  name: 'relay'
  params: {
    containerGroupName: 'ci-${nameSuffix}'
    location: location
    image: relayImage
    registryLoginServer: registry.outputs.loginServer
    identityId: identity.outputs.identityId
    dnsNameLabel: dnsNameLabel
    udpPort: udpPort
    cpu: cpu
    memoryGb: memoryGb
    relayEnvironment: relayEnvironment
    logAnalyticsWorkspaceName: logs.outputs.workspaceName
    tags: resourceTags
  }
}

module budget 'modules/budget.bicep' = if (!empty(alertEmail)) {
  name: 'budget'
  params: {
    budgetName: 'budget-${nameSuffix}'
    monthlyBudgetUsd: monthlyBudgetUsd
    startDate: budgetStartDate
    alertEmail: alertEmail
  }
}

@description('FQDN of the relay; empty when deployRelay is false.')
output relayFqdn string = deployRelay ? relay!.outputs.fqdn : ''

@description('Public IP of the relay; empty when deployRelay is false. It can change on a re-create, so clients use the FQDN.')
output relayIp string = deployRelay ? relay!.outputs.ip : ''

@description('<fqdn>:<port> that clients connect to; empty when deployRelay is false.')
output relayEndpoint string = deployRelay ? '${relay!.outputs.fqdn}:${udpPort}' : ''

@description('Login server of the environment registry.')
output acrLoginServer string = registry.outputs.loginServer

@description('Name of the environment registry.')
output acrName string = registry.outputs.registryName

@description('Principal id of the managed identity that pulls the image.')
output identityPrincipalId string = identity.outputs.principalId
