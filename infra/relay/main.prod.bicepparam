using './main.bicep'

// Production relay: the client default. Promoted by digest from dev behind the relay-prod approval.
// REPLACE_ME values are user-specific: set them in this file or, for CI, through the environment variables read below.

param location = 'southcentralus'
param environmentName = 'prod'
param namePrefix = 'wpiemu-relay'

// Unique DNS label suffix (ACI labels are global per region), e.g. '47c926'. FQDN: wpiemu-relay<suffix>.southcentralus.azurecontainer.io
param dnsLabelSuffix = readEnvironmentVariable('WPIEMU_RELAY_DNS_SUFFIX', 'REPLACE_ME')

// First pass creates the registry only. The pipeline passes the real image digest and deployRelay=true.
param relayImage = ''
param deployRelay = false

param udpPort = 7777
param cpu = '0.5'
param memoryGb = '1.0'

param maxRooms = 100
param roomTtlMs = 30000
param peerBytesPerSec = 65536
param roomBytesPerSec = 0

param logRetentionDays = 90
param dailyLogCapGb = '0.5'

param alertEmail = readEnvironmentVariable('WPIEMU_RELAY_ALERT_EMAIL', 'REPLACE_ME')
param monthlyBudgetUsd = 60

param tags = {
  costCenter: 'wpiemu-relay-prod'
}
