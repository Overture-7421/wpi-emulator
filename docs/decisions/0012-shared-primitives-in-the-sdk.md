# Shared primitives live in the SDK

Read this before copying a helper, table or rule between modules.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Rationale
Copies of the same rule can diverge in units, thresholds or edge behavior. Where several modules
need the same platform-neutral contract, one implementation is easier to review and test.

## Decision
Put small, standard-library-only shared contracts and primitives in the header-only SDK. Keep
platform effects and subsystem policies with their owners; share a helper only when behavior is
actually equivalent.

## Trade-offs
The SDK gains compile-time and contract surface, so shared code should stay small and changes
need focused equivalence tests. Similar-looking code is not proof of identical policy.

Current module boundaries and coding rules are in [Architecture](../ARCHITECTURE.md) and
[Contributing](../CONTRIBUTING.md).
