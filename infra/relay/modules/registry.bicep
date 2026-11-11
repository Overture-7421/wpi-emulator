// Container registry that holds the relay image of one environment.
// Basic SKU: the relay image is small and pulled rarely. Admin user and anonymous pull are off; the container
// group pulls with a managed identity that has AcrPull (see identity.bicep). Public network access stays on
// because Azure Container Instances pull over the public endpoint (Basic has no private link).

targetScope = 'resourceGroup'

@description('Registry name: 5-50 lower-case letters and digits, globally unique (the caller derives it from uniqueString).')
@minLength(5)
@maxLength(50)
param registryName string

@description('Azure region of the registry.')
param location string

@description('Tags applied to the registry.')
param tags object

resource registry 'Microsoft.ContainerRegistry/registries@2025-04-01' = {
  name: registryName
  location: location
  tags: tags
  sku: {
    name: 'Basic'
  }
  properties: {
    adminUserEnabled: false
    anonymousPullEnabled: false
    publicNetworkAccess: 'Enabled'
    networkRuleBypassOptions: 'AzureServices'
  }
}

output registryId string = registry.id
output registryName string = registry.name
output loginServer string = registry.properties.loginServer
