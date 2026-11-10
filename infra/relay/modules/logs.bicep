// Log Analytics workspace that receives the relay's stdout (JSON lines, never frame contents).
// Retention and the daily ingestion cap bound what logging can cost.

targetScope = 'resourceGroup'

@description('Workspace name.')
param workspaceName string

@description('Azure region of the workspace.')
param location string

@description('Days the logs are kept.')
@minValue(30)
@maxValue(730)
param retentionInDays int

@description('Daily ingestion cap in GB (a decimal string such as "0.5"). Ingestion stops for the rest of the day once it is reached.')
param dailyQuotaGb string

@description('Tags applied to the workspace.')
param tags object

resource workspace 'Microsoft.OperationalInsights/workspaces@2025-02-01' = {
  name: workspaceName
  location: location
  tags: tags
  properties: {
    sku: {
      name: 'PerGB2018'
    }
    retentionInDays: retentionInDays
    workspaceCapping: {
      dailyQuotaGb: json(dailyQuotaGb)
    }
    publicNetworkAccessForIngestion: 'Enabled'
    publicNetworkAccessForQuery: 'Enabled'
  }
}

output workspaceId string = workspace.id
output workspaceName string = workspace.name
