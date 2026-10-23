# One host contract

Read this when an app page needs runtime access or a new app library is proposed.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Rationale
Parallel host interfaces and adapters copy process state between the shell and run view and can
drift. App layering should also let editor and run pages share a contract without letting a page
select or replace the active project.

## Decision
Use one `IRuntimeHost` seam for runtime-facing pages, with project selection owned by app
composition rather than by a page.

## Trade-offs
Shared app support and include ownership need explicit maintenance, but there is one runtime
contract to fake and evolve.

Current app ownership is in [Application](../reference/app.md).
