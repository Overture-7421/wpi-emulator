using './main.bicep'

// Development relay (test traffic, probes, preview client builds). Deployed from main by relay-deploy.
// REPLACE_ME values are user-specific: set them in this file or, for CI, through the environment variables read below.

param location = 'southcentralus'
param environmentName = 'dev'
param namePrefix = 'wpiemu-relay'

// Unique DNS label suffix (ACI labels are global per region), e.g. '-a1b2'. FQDN: wpiemu-relay-dev<suffix>.southcentralus.azurecontainer.io
param dnsLabelSuffix = readEnvironmentVariable('WPIEMU_RELAY_DNS_SUFFIX', 'REPLACE_ME')

// First pass creates the registry only. The pipeline passes the real image digest and deployRelay=true.
param relayImage = ''
param deployRelay = false

param udpPort = 7777
param cpu = '0.5'
param memoryGb = '1.0'

param maxRooms = 20
param roomTtlMs = 30000
param peerBytesPerSec = 65536
param roomBytesPerSec = 0

param logRetentionDays = 30
param dailyLogCapGb = '0.2'

param alertEmail = readEnvironmentVariable('WPIEMU_RELAY_ALERT_EMAIL', 'REPLACE_ME')
param monthlyBudgetUsd = 25

param tags = {
  costCenter: 'wpiemu-relay-dev'
}
