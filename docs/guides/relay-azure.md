# Relay on Azure

**Read this when:** deploying, promoting, rolling back or operating the practice-match relay, or
self-hosting it.
**Prerequisites:** [Match network](../reference/match-network.md#relay-protocol) for what the relay
does and does not see; [Building](../BUILDING.md) for the relay-only build.

The relay is a small UDP server (`wpiemu_relay`) that forwards encrypted frames between a match host
and its clients. It runs as one Azure Container Instances (ACI) container group per environment. Azure
Container Apps was rejected because it offers only HTTP and TCP ingress; ACI supports UDP. Everything
here lives in [`infra/relay/`](../../infra/relay), [`relay/`](../../relay) and the `relay-*` workflows
in [`.github/workflows/`](../../.github/workflows). There are no stored secrets anywhere: the relay
has none, and the pipelines use GitHub OIDC.

## Environments

| | dev | prod |
|---|---|---|
| Resource group | `rg-wpiemu-relay-dev` | `rg-wpiemu-relay-prod` |
| DNS label | `wpiemu-relay-dev<suffix>` | `wpiemu-relay<suffix>` |
| FQDN | `<label>.southcentralus.azurecontainer.io` | the same form |
| GitHub Environment | `relay-dev` (no approval) | `relay-prod` (required reviewers, `main` only) |
| Deploys | automatically from `main`; used for tests, probes and preview clients | only by promoting a digest that passed verification in dev |
| Room cap, log cap, budget (parameter files) | 20 rooms, 0.2 GB/day, 25 USD | 100 rooms, 0.5 GB/day, 60 USD |

Each environment has its own resource group, registry, managed identity, logs and relay. The game
client and host UI default to prod; Settings has a Development and a custom `host:port` choice (see
[Application](../reference/app.md#the-practice-match-panel)). A client always uses the FQDN: a
re-created container can get another IP.

## What the Bicep deploys

[`main.bicep`](../../infra/relay/main.bicep) deploys one environment into one resource group:

| Module | Resource |
|---|---|
| `registry` | Container Registry, Basic SKU, admin user and anonymous pull off |
| `identity` | User-assigned managed identity with the AcrPull role on the registry |
| `logs` | Log Analytics workspace with retention (30 days or more) and a **daily ingestion cap** |
| `relay` | ACI Linux container group, 0.5 vCPU and 1 GB by default, public IP, **one UDP port**, DNS label, restart `Always`, `WPIEMU_RELAY_*` environment variables, exec liveness probe `wpiemu_relay --selfcheck` (every 30 s, 3 failures restart), diagnostics to Log Analytics |
| `budget` | Monthly budget with an email at 80 percent actual and 100 percent forecast, created only when `alertEmail` is set |

The image is pulled by **digest** with the managed identity. Parameters: `location` (default
`southcentralus`), `environmentName`, `dnsLabelSuffix` (ACI labels are global per region, so start the
suffix with a hyphen), `relayImage`, `deployRelay`, `udpPort` (default 7777), `cpu`, `memoryGb` and the
relay limits. `deployRelay=false` creates the registry, identity, logs and budget so an image can be
pushed first; the second pass passes `relayImage` and creates the container group. The label reuse
policy is `Unsecure`, the only one that keeps the name `<label>.southcentralus.azurecontainer.io`
(every other policy inserts a hash label, which the apps' relay presets do not know). Anyone can claim
the label after the relay is deleted; that cannot expose game data, which is encrypted end to end, but
it could deny service, so deleting a relay retires its name. A container group's policy cannot change
in place: delete the group and deploy again.
[`main.dev.bicepparam`](../../infra/relay/main.dev.bicepparam) and
[`main.prod.bicepparam`](../../infra/relay/main.prod.bicepparam) read `WPIEMU_RELAY_DNS_SUFFIX` and
`WPIEMU_RELAY_ALERT_EMAIL` from the environment.

Only the relay port is open, over UDP. Port 7777 is the default. UDP 443 is often allowed on school and
corporate networks that block other UDP ports, but binding a port below 1024 as the container's non-root
user needs a capability ACI cannot add; read the note in [`relay/Dockerfile`](../../relay/Dockerfile)
and test it before choosing 443. The relay is a single instance with no high availability, and an update
restarts the container (tens of seconds); clients rejoin through their resume token.

## The image

[`relay/Dockerfile`](../../relay/Dockerfile) builds a relay-only tree (`WPIEMU_RELAY_ONLY`: sdk, ipc, net
and relay, so no Box3D, Qt or WPILib) on Ubuntu 24.04 with GCC 14, hardening flags and warnings as errors,
links the C++ runtime statically, and runs as uid 10001 with only `wpiemu_relay` and `wpiemu_relay_probe`.
Build and smoke-test it locally with Docker and PowerShell 7:

```powershell
pwsh relay/tests/container_smoke.ps1
```

The script builds the image, runs it, waits for `--selfcheck`, runs the round-trip probe inside the
container, stops it with SIGTERM and asserts a clean exit, then removes the container and image.

## Pipelines

All workflows use top-level `permissions: contents: read`; `id-token: write` is granted only to the jobs
that log in to Azure, and the Azure client, tenant and subscription ids are repository or environment
**variables** (identifiers, not secrets).

| Workflow | Trigger | What it does |
|---|---|---|
| `relay-ci` | Pull request and push to `main` touching `relay/`, `net/`, `sdk/`, `ipc/`, `infra/relay/`, CMake, `versions.json`, `.dockerignore` or `relay-*` workflows | `offline`: workflow lint tests, `actionlint`, `bicep.tests.ps1` (Bicep build and lint of both parameter files), `preflight.tests.ps1` and `bootstrap.tests.ps1` (Azure and `gh` faked). `relay-tests`: relay-only Linux build, then `wpiemu_relay_tests` and `wpiemu_net_tests`. `container`: hadolint, build, smoke test, Trivy. `whatif`: `az deployment group what-if` against dev, for same-repository pull requests only and only once `AZURE_PR_CLIENT_ID` exists; a preflight runs first as an advisory |
| `relay-container` | Called by `relay-ci` and `relay-deploy` | Lint, build, smoke-test and scan; holds no credentials. With `save_image` it uploads the scanned image so the deploy pushes exactly what was scanned |
| `relay-deploy` | Push to `main` on the same paths (deploys **dev**), or `workflow_dispatch` with `environment` (`dev` or `prod`) and `image_digest` | `plan` (refuses any ref but `main`; prod must come with a digest that has a dev verification record), `build` (when no digest was given), then `deploy-dev` or `deploy-prod` |
| `relay-deploy-environment` | Called by `relay-deploy` | One job bound to the GitHub Environment, so the `relay-prod` approval is asked once |

The per-environment job runs: Azure login (OIDC, that environment's identity); infra with
`deployRelay=false` (creates the registry); read the previous known-good digest; put the image into **this
environment's** registry (push the scanned image, or for prod `az acr import` the verified digest from dev,
which copies the manifest unchanged); deploy the relay **by digest**; verify by running the deployed image's
`wpiemu_relay_probe` from the runner (reach until the relay answers, then a full round trip of 20 echoes and
a 64 KiB reliable payload); record the digest as known good; and, in dev, upload the `relay-verified-<digest>`
artifact (90 days) that lets prod promote it. Concurrency is one run per environment and never cancels a
deploy in progress. **Prod never rebuilds.**

The probe is also usable by hand. `--mode reach` joins the reserved ping room and creates nothing;
`--mode full` registers a room, joins it, runs the encrypted handshake and exchanges messages on both
delivery classes; `--json` prints one line. For example:

```powershell
wpiemu_relay_probe --relay wpiemu-relay-dev<suffix>.southcentralus.azurecontainer.io:7777 --mode full --json
```

It exits 0 on success, 1 on failure and 2 on a usage error; `WPIEMU_RELAY_PROBE_RELAY` can replace
`--relay`.

## One-time setup and first deployment

Run these in order, once per subscription and repository, as a person who owns both. The scripts need
PowerShell 7 and the Azure CLI; no script logs in for you. Step 1 is read-only; steps 3, 5 and 6 change
Azure and GitHub, so a person runs them, never an agent or CI.

1. **Check the region (read-only).** After `az login`, run `pwsh infra/relay/preflight.ps1`. It checks the
   account, that the location exists, that the resource providers (Container Instance, Container Registry,
   Operational Insights, Consumption, Managed Identity) are registered, that each resource type is offered
   in the region, quota headroom for the relay and the UDP port limit, and Bicep tooling. It prints the
   exact `az provider register` commands it needs, or registers with `-Register`, and lists nearby regions
   when the chosen one fails. DNS-label uniqueness and live capacity can only be proven by deploying.
2. **Bootstrap dry run.** `pwsh infra/relay/bootstrap.ps1` prints every command it would run and changes
   nothing.
3. **Bootstrap for real.**
   `pwsh infra/relay/bootstrap.ps1 -Apply -ConfigureGitHub -ProdReviewers <login> -AlertEmail <address>`
   (after `gh auth login`). It creates both resource groups; **three** Entra app registrations with
   one GitHub OIDC federated credential each (`wpiemu-relay-deploy-dev` for `environment:relay-dev`,
   `wpiemu-relay-deploy-prod` for `environment:relay-prod`, and `wpiemu-relay-whatif` for pull requests,
   each with the subject form GitHub emits for the repository: see Troubleshooting);
   Contributor plus a role-assignment right limited by an ABAC condition to AcrPull, each deploy identity
   on its **own** resource group only; the what-if identity with Reader plus a custom what-if role on dev;
   and the GitHub environments and variables below. One app per environment matters: a token only has to
   match a credential's subject, so an app that could deploy to both groups would let an unapproved
   `relay-dev` job reach prod.
4. **First dev deployment.** Push to `main` (or run `relay-deploy` with `environment` dev). It creates the
   dev registry, pushes the image, deploys and probes it. Open the run summary for the digest.
5. **Grant promotion.** `pwsh infra/relay/bootstrap.ps1 -Apply -Step GrantPromotion` gives the prod deploy
   identity AcrPull on the dev registry so `az acr import` can copy an image. It is skipped until the
   dev registry exists, hence this order.
6. **Prod.** Run `relay-deploy` manually with `environment` prod and `image_digest` set to the digest that
   passed verification in dev. A reviewer approves the `relay-prod` environment, then the digest is imported,
   deployed and probed.
7. **Point the apps at it.** Set `RELAY_DNS_SUFFIX` (the same suffix bootstrap chose) where the app reads it
   (see [Application](../reference/app.md#the-practice-match-panel)).

Variables that bootstrap sets:

| Scope | Variables |
|---|---|
| Environment `relay-dev` and `relay-prod` (each with its own values) | `AZURE_CLIENT_ID` (that environment's app), `AZURE_TENANT_ID`, `AZURE_SUBSCRIPTION_ID`, `RELAY_RESOURCE_GROUP`, `RELAY_DNS_SUFFIX`, `RELAY_ALERT_EMAIL` |
| Repository (for the what-if job; omitted with `-SkipPrIdentity`) | `AZURE_LOCATION`, `AZURE_PR_CLIENT_ID`, `AZURE_TENANT_ID`, `AZURE_SUBSCRIPTION_ID`, `RELAY_PR_RESOURCE_GROUP`, `RELAY_DNS_SUFFIX`, `RELAY_ALERT_EMAIL` |

`-Remove` tears down what the script created (a dry run until `-Apply`). `relay-prod` required reviewers
need a repository plan that supports them.

## Promote, roll back, operate

**Day to day.** A change to the relay, `net/`, `sdk/`, `ipc/` or `infra/relay/` that lands on `main` redeploys
dev. To release: run `relay-deploy` with `environment` prod and the dev-verified digest. The record expires
after 90 days; deploy that digest to dev again to refresh it.

**Rollback.** Each successful deploy tags the resource group with `relayKnownGoodDigest`. When a deployment
or its verification fails, the job redeploys the previous known-good digest by itself and probes it. To roll
back by hand, read the tag (`az group show --name <group> --query tags.relayKnownGoodDigest -o tsv`) and
run `relay-deploy` again for that environment with it as `image_digest`; prod accepts it while its
verification record is still valid.

### Configure and operate

The relay reads command-line options, overridden by environment variables
([`relay_options.h`](../../relay/include/wpiemu/relay/relay_options.h)); an invalid value is an error that
names the setting. Exit codes: 0 ok, 1 runtime failure or failed `--selfcheck`, 2 bad configuration.

| Variable | Default | Meaning |
|---|---|---|
| `WPIEMU_RELAY_PORT` | 7777 | UDP port; also the port `--selfcheck` asks |
| `WPIEMU_RELAY_BIND` | `::` | Address to bind (dual stack) |
| `WPIEMU_RELAY_MAX_ROOMS` | 100 | Rooms at once (up to 500) |
| `WPIEMU_RELAY_MAX_PEERS` | 6 | Clients per room (up to 6) |
| `WPIEMU_RELAY_ROOM_TTL_MS` | 30000 | How long a room survives without its host |
| `WPIEMU_RELAY_PEER_BYTES_PER_SEC` | 65536 | Per-client token bucket |
| `WPIEMU_RELAY_ROOM_BYTES_PER_SEC` | 0 | Per-room forwarded bytes; 0 derives it from the peer rate |
| `WPIEMU_RELAY_LOCKOUT_FAILURES` | 10 | Failed joins or registrations from one source before lockout; 0 disables it |
| `WPIEMU_RELAY_LOCKOUT_WINDOW_S` | 60 | Window in which they count |
| `WPIEMU_RELAY_LOCKOUT_DURATION_S` | 300 | How long a source is refused |
| `WPIEMU_RELAY_STATS_INTERVAL_S` | 60 | Period of the `stats` log line; 0 disables it |
| `WPIEMU_RELAY_LOG` | `json` | `json` or `text` |

The Bicep sets port, rooms, room TTL, both byte rates and the log format; the other settings keep their
defaults. Logs are JSON lines (start, room events, periodic stats, stop) in Log Analytics; they carry room
ids and peer addresses but never frame contents. Cost is dominated by the container group, the Basic
registry, egress and logs; the log cap and the budget alert bound surprises. Check the Azure pricing
calculator for current numbers.

## Self-host

Anyone can run the relay: build the image (above) or the `wpiemu_relay` target, run it with the UDP port
published, for example `docker run --rm -p 7777:7777/udp wpiemu-relay:dev`, and in the host app's
Practice match panel choose the custom relay `host:port`. Players use the same invite. The relay needs
a reachable UDP port and nothing else: no database and no secrets. Publishing the image for self-hosters
is not set up.

## Troubleshooting

- **A host cannot open a relay room and the probe times out:** the container may be restarting, the
  DNS label may differ (`RELAY_DNS_SUFFIX`), or the player's network blocks outbound UDP. Run
  `wpiemu_relay_probe --mode reach` from that network.
- **`RoomRejected(RoomTaken)`:** another host owns the room id; the host picks a new invite.
- **`LockedOut`:** that address had 10 failed joins in 60 s; it clears after 5 minutes.
- **A pipeline's Azure step cannot log in:** the environment's `AZURE_CLIENT_ID` must be the app whose
  federated subject is that environment; re-run `bootstrap.ps1` as a dry run to compare.
- **`AADSTS700213: No matching federated identity record found for presented assertion subject`:** the
  message names the subject GitHub presented. GitHub emits the immutable form
  `repo:<owner>@<ownerId>/<repo>@<repoId>:...` for repositories created on or after 2026-07-15 and for older
  ones that opted in; a credential made for `repo:<owner>/<repo>:...` never matches it. `bootstrap.ps1`
  reads the ids with `gh api repos/<repository>` and picks the form (`-SubjectFormat Auto`); without `gh`
  it falls back to the name-based form with a warning. Re-run it with `-SubjectFormat Immutable` (and
  `-OwnerId`/`-RepositoryId` when `gh` is unavailable): it updates the credentials in place. If the subject
  is right and the error remains, wait a minute: new credentials take time to propagate.
- **The what-if job fails with `Authorization failed for template resource ... Microsoft.Resources/deployments/write`
  (or `.../registries/write` and similar):** a what-if needs the permissions of a real deployment unless it
  runs with `--validation-level ProviderNoRbac`, which checks read access only (Azure CLI 2.76 or later).
  `relay-ci` passes that flag so the what-if identity can stay read-only (Reader plus the custom what-if role
  on the dev resource group); do not widen that identity to make a what-if pass.
- **Region or quota failure in `preflight.ps1`:** it lists the nearest passing regions; changing the
  region changes `location` in the parameter files and the `AZURE_LOCATION` variable.

Next: [Match network](../reference/match-network.md).
