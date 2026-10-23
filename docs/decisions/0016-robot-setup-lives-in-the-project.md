# Robot setup lives in the project

Read this when changing where a robot setup is persisted.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Rationale
Robot configuration describes a project and should travel with it. Keeping an editable project
copy and a second app-data copy creates competing authorities: a teammate can pull the project
from Git and silently replace newer settings with stale configuration, losing mappings and model
edits.

## Decision
Keep one authoritative setup at `<project>/.wpiemu/robot.json`; do not maintain a live duplicate
in application storage.

## Trade-offs
The setup is project-local and can be shared or merged with the robot project, so teams must decide
how that file participates in version control. Runtime owns legacy migration, backup and error
behavior; this record does not define those procedures.

See the [runtime configuration reference](../reference/runtime.md#persistence-responsibilities).
