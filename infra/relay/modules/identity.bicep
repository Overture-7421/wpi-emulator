// User-assigned managed identity the relay container group uses to pull its image, plus its AcrPull role
// assignment on the registry. The role assignment name is deterministic, so re-deploys are idempotent.

targetScope = 'resourceGroup'

@description('Identity name.')
param identityName string

@description('Azure region of the identity.')
param location string

@description('Name of the registry (in this resource group) the identity may pull from.')
param registryName string

@description('Tags applied to the identity.')
param tags object

// Built-in role "AcrPull".
var acrPullRoleDefinitionId = subscriptionResourceId('Microsoft.Authorization/roleDefinitions', '7f951dda-4ed3-4680-a7ca-43fe172d538d')

resource registry 'Microsoft.ContainerRegistry/registries@2025-04-01' existing = {
  name: registryName
}

resource identity 'Microsoft.ManagedIdentity/userAssignedIdentities@2024-11-30' = {
  name: identityName
  location: location
  tags: tags
}

resource acrPull 'Microsoft.Authorization/roleAssignments@2022-04-01' = {
  scope: registry
  name: guid(registry.id, identity.id, acrPullRoleDefinitionId)
  properties: {
    principalId: identity.properties.principalId
    principalType: 'ServicePrincipal'
    roleDefinitionId: acrPullRoleDefinitionId
  }
}

output identityId string = identity.id
output principalId string = identity.properties.principalId
output acrPullAssignmentId string = acrPull.id
