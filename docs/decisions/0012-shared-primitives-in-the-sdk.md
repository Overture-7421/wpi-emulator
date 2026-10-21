# Shared primitives live in the SDK

Read this when you are about to copy a helper, table or rule between modules.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Context
A consolidation audit found the same rule stated in several modules: property builders and value
readers, validation rules for zones, cameras and sensors, wheel layouts, wire field lists, queue
and thread loops, test waits and fakes. Copies had already diverged in thresholds, severities
and edge behavior.

## Decision
What several modules need and can express with the standard library is defined once in the
header-only SDK and derived from there. Test doubles and helpers that need no platform call
live in the SDK testing headers; ones that do, such as the environment guard, live in
`tests/support`; Qt code shared by the page libraries lives in `app/ui`. Wire structs list their
fields once and one description drives both encode and decode. Validation rules take a small
catalog interface so core and the editor run the same code.

Policy stays local: queue and thread primitives are shared, but watchdogs, drop rules and
shutdown order stay with their owner. Look-alikes are merged only with proof that behavior
is unchanged; the deliberate differences are listed in Contributing.

## Consequences
One definition per rule, so a change cannot reach only some modules. The SDK grows and every
includer pays header compile time, so shared code stays small and inline. A shared helper is a
dependency: changing it needs golden or equivalence tests, which the audit added first.

Next: [User guide](../USER_GUIDE.md).
