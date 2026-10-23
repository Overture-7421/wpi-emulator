# Plugin-owned season data

Read this when season, field or tag data appears to belong in core, a bridge or the app.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Rationale
Embedding a season's dimensions, tags or identifiers in shared consumers turns each new game year
into a search across core, bridges and UI. A plugin boundary lets those facts travel with the rules
that own them.

## Decision
Keep season-specific field, tag, zone and game-value data in the game plugin and expose it to
consumers through SDK contracts. Shared facts belong in SDK only when they are truly season-neutral.

## Trade-offs
A plugin must provide the metadata needed by generic consumers; missing season data cannot be
repaired safely by a shared bridge inventing a default.

Current ownership is described in [Game plugins](../reference/game-plugin.md).
