// Cost alert for the relay's resource group: a monthly budget with an email at 80% of actual and 100% of forecast spend.
// Requires the Microsoft.Consumption resource provider to be registered in the subscription.

targetScope = 'resourceGroup'

@description('Budget name.')
param budgetName string

@description('Monthly budget in USD.')
@minValue(1)
param monthlyBudgetUsd int

@description('First day of the month the budget starts, as yyyy-MM-01 (Azure requires the first of a month).')
param startDate string

@description('Address that receives the budget notifications.')
@minLength(3)
param alertEmail string

resource budget 'Microsoft.Consumption/budgets@2026-06-01' = {
  name: budgetName
  properties: {
    category: 'Cost'
    amount: monthlyBudgetUsd
    timeGrain: 'Monthly'
    timePeriod: {
      startDate: startDate
    }
    notifications: {
      Actual_GreaterThan_80_Percent: {
        enabled: true
        operator: 'GreaterThan'
        threshold: 80
        thresholdType: 'Actual'
        contactEmails: [
          alertEmail
        ]
      }
      Forecasted_GreaterThan_100_Percent: {
        enabled: true
        operator: 'GreaterThan'
        threshold: 100
        thresholdType: 'Forecasted'
        contactEmails: [
          alertEmail
        ]
      }
    }
  }
}

output budgetId string = budget.id
