"""Lints the relay workflows (.github/workflows/relay-*.yml) against the facts they depend on.

Offline and stdlib + PyYAML only. The environment names, the variable names and the federated-credential subjects are read from
infra/relay/bootstrap.ps1, the Bicep parameter variables from infra/relay/main.*.bicepparam, the probe options from the relay
sources, so a rename in one place fails here instead of in a deployment.

Run: python -m unittest discover -s infra/relay/tests -p "test_*.py" -v
"""
import json
import re
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
WORKFLOWS = ROOT / ".github" / "workflows"
RELAY_FILES = sorted(WORKFLOWS.glob("relay-*.yml"))
CI = WORKFLOWS / "relay-ci.yml"
CONTAINER = WORKFLOWS / "relay-container.yml"
DEPLOY = WORKFLOWS / "relay-deploy.yml"
ENVIRONMENT = WORKFLOWS / "relay-deploy-environment.yml"

REQUIRED_PATHS = {
    "relay/**", "net/**", "sdk/**", "ipc/**", "infra/relay/**", ".github/workflows/relay-*.yml",
}
DIGEST_IMAGE = re.compile(r"@\$\{\{ steps\.(image|known_good)\.outputs\.digest \}\}$")


def load(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def triggers(doc):
    # PyYAML reads the bare key `on` as the boolean True.
    return doc["on"] if "on" in doc else doc[True]


def jobs(doc):
    return doc["jobs"].items()


def logs_in(job):
    return any(str(step.get("uses", "")).startswith("azure/login@") for step in job.get("steps", []))


def job_text(job):
    return json.dumps(job)


def variables_of(text):
    return set(re.findall(r"\bvars\.([A-Z][A-Z0-9_]*)", text))


def bootstrap_facts():
    text = (ROOT / "infra" / "relay" / "bootstrap.ps1").read_text(encoding="utf-8")
    envs = re.search(r"DevEnvironment = '([^']+)'; ProdEnvironment = '([^']+)'", text)
    section = text[text.index("function Get-GitHubVariables"):text.index("function Get-ReviewerEntries")]
    environment_level = set(re.findall(r"\$v\['(\w+)'\]", section))
    repository_level = set(re.findall(r"\$repo\['(\w+)'\]", section))
    repository_level |= set(re.findall(r"\$repo = \[ordered\]@\{ (\w+) =", section))
    return {
        "dev": envs.group(1),
        "prod": envs.group(2),
        "environment_level": environment_level,
        "repository_level": repository_level,
        "text": text,
    }


class WorkflowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.docs = {path.name: load(path) for path in RELAY_FILES}
        cls.raw = {path.name: path.read_text(encoding="utf-8") for path in RELAY_FILES}
        cls.bootstrap = bootstrap_facts()

    def all_jobs(self):
        for name, doc in self.docs.items():
            for job_id, job in jobs(doc):
                yield name, job_id, job

    def test_the_workflows_exist(self):
        self.assertEqual({p.name for p in RELAY_FILES},
                         {"relay-ci.yml", "relay-container.yml", "relay-deploy.yml", "relay-deploy-environment.yml"})

    def test_bootstrap_publishes_what_the_workflows_need(self):
        self.assertEqual(self.bootstrap["dev"], "relay-dev")
        self.assertEqual(self.bootstrap["prod"], "relay-prod")
        self.assertIn(":pull_request", self.bootstrap["text"])
        self.assertIn("environment:$($o.DevEnvironment)", self.bootstrap["text"])
        self.assertIn("environment:$($o.ProdEnvironment)", self.bootstrap["text"])

    def test_no_pull_request_target_and_no_workflow_run(self):
        for name, text in self.raw.items():
            self.assertNotIn("pull_request_target", text, name)
            self.assertNotRegex(text, r"(?m)^\s*workflow_run\s*:", name)

    def test_no_stored_secrets_and_no_az_login(self):
        for name, text in self.raw.items():
            self.assertNotRegex(text, r"\bsecrets\.(?!GITHUB_TOKEN)", name)
            self.assertNotRegex(text, r"\baz\s+login\b", name)
            self.assertNotIn("client-secret", text, name)
            self.assertNotRegex(text, r"(?m)^\s*creds\s*:", name)

    def test_logins_are_oidc_with_variables(self):
        for name, job_id, job in self.all_jobs():
            for step in job.get("steps", []):
                if str(step.get("uses", "")).startswith("azure/login@"):
                    with_ = step["with"]
                    self.assertEqual(set(with_), {"client-id", "tenant-id", "subscription-id"}, f"{name}:{job_id}")
                    for value in with_.values():
                        self.assertTrue(str(value).startswith("${{ vars."), f"{name}:{job_id} {value}")

    def test_actions_are_pinned_to_a_major_version(self):
        local = re.compile(r"^\./\.github/workflows/relay-[\w-]+\.yml$")
        remote = re.compile(r"^[\w.-]+/[\w.-]+@v\d+$")
        for name, job_id, job in self.all_jobs():
            uses = [job["uses"]] if "uses" in job else []
            uses += [s["uses"] for s in job.get("steps", []) if "uses" in s]
            for ref in uses:
                self.assertTrue(local.match(ref) or remote.match(ref), f"{name}:{job_id} {ref}")

    def test_permissions_are_least_privilege(self):
        for name, doc in self.docs.items():
            self.assertEqual(doc["permissions"], {"contents": "read"}, name)
        for name, job_id, job in self.all_jobs():
            permissions = job.get("permissions", {})
            self.assertNotIn(permissions, ("write-all", "read-all"), f"{name}:{job_id}")
            for scope, level in permissions.items():
                if scope == "id-token":
                    continue
                self.assertEqual(level, "read", f"{name}:{job_id} {scope}")
            needs_token = logs_in(job) or job.get("uses", "").endswith("relay-deploy-environment.yml")
            self.assertEqual(permissions.get("id-token") == "write", needs_token, f"{name}:{job_id} id-token")

    def test_every_job_has_a_timeout(self):
        for name, job_id, job in self.all_jobs():
            if "uses" not in job:
                self.assertIsInstance(job.get("timeout-minutes"), int, f"{name}:{job_id}")

    def test_script_steps_never_interpolate_expressions(self):
        for name, job_id, job in self.all_jobs():
            for step in job.get("steps", []):
                self.assertNotIn("${{", str(step.get("run", "")), f"{name}:{job_id} {step.get('name')}")

    def test_environment_jobs_use_the_bootstrap_names_exactly(self):
        deploy = self.docs["relay-deploy.yml"]["jobs"]
        self.assertEqual(deploy["deploy-dev"]["with"]["environment"], self.bootstrap["dev"])
        self.assertEqual(deploy["deploy-prod"]["with"]["environment"], self.bootstrap["prod"])
        callee = self.docs["relay-deploy-environment.yml"]["jobs"]["deploy"]
        self.assertEqual(callee["environment"], "${{ inputs.environment }}")
        for name, job_id, job in self.all_jobs():
            literal = job.get("environment")
            if isinstance(literal, str) and not literal.startswith("${{"):
                self.assertIn(literal, (self.bootstrap["dev"], self.bootstrap["prod"]), f"{name}:{job_id}")

    def test_every_job_that_deploys_binds_an_environment(self):
        for name, job_id, job in self.all_jobs():
            deploys = "AZURE_CLIENT_ID" in variables_of(job_text(job)) or str(job.get("uses", "")).endswith(
                "relay-deploy-environment.yml")
            if deploys:
                bound = "environment" in job or "environment" in job.get("with", {})
                self.assertTrue(bound, f"{name}:{job_id} deploys without an environment")

    def test_variables_are_the_ones_bootstrap_publishes_at_the_right_scope(self):
        env_level = self.bootstrap["environment_level"]
        repo_level = self.bootstrap["repository_level"]
        published = env_level | repo_level
        env_only = env_level - repo_level
        pr_only = {v for v in repo_level if "_PR_" in v}
        for name, job_id, job in self.all_jobs():
            used = variables_of(job_text(job))
            self.assertLessEqual(used, published, f"{name}:{job_id} reads a variable bootstrap does not publish")
            has_environment = "environment" in job
            if not has_environment and "uses" not in job:
                self.assertFalse(used & env_only, f"{name}:{job_id} reads environment-level {used & env_only} without an environment")
            if has_environment:
                self.assertFalse(used & pr_only, f"{name}:{job_id} reads pull-request variables in an environment")
            if used & pr_only:
                self.assertEqual((name, job_id), ("relay-ci.yml", "whatif"))

    def test_what_if_never_runs_for_forks_and_uses_the_pull_request_identity(self):
        whatif = self.docs["relay-ci.yml"]["jobs"]["whatif"]
        condition = " ".join(str(whatif["if"]).split())
        self.assertIn("github.event_name == 'pull_request'", condition)
        self.assertIn("github.event.pull_request.head.repo.full_name == github.repository", condition)
        self.assertNotIn("environment", whatif)
        self.assertEqual(whatif["permissions"], {"contents": "read", "id-token": "write"})
        login = next(s for s in whatif["steps"] if str(s.get("uses", "")).startswith("azure/login@"))
        self.assertEqual(login["with"]["client-id"], "${{ vars.AZURE_PR_CLIENT_ID }}")
        for job_id, job in jobs(self.docs["relay-ci.yml"]):
            if job_id != "whatif":
                self.assertNotIn("id-token", job.get("permissions", {}), job_id)
        self.assertNotIn("az deployment group create", self.raw["relay-ci.yml"])
        self.assertIn("what-if", self.raw["relay-ci.yml"])

    def test_ci_triggers_and_paths(self):
        on = triggers(self.docs["relay-ci.yml"])
        self.assertEqual(set(on), {"pull_request", "push"})
        self.assertEqual(on["push"]["branches"], ["main"])
        for event in ("pull_request", "push"):
            self.assertLessEqual(REQUIRED_PATHS, set(on[event]["paths"]), event)

    def test_deploy_triggers(self):
        on = triggers(self.docs["relay-deploy.yml"])
        self.assertEqual(set(on), {"push", "workflow_dispatch"})
        self.assertEqual(on["push"]["branches"], ["main"])
        self.assertEqual(set(on["push"]["paths"]), set(triggers(self.docs["relay-ci.yml"])["push"]["paths"]))
        inputs = on["workflow_dispatch"]["inputs"]
        self.assertEqual(inputs["environment"]["options"], ["dev", "prod"])
        self.assertIn("image_digest", inputs)
        plan = self.docs["relay-deploy.yml"]["jobs"]["plan"]
        self.assertIn("refs/heads/main", job_text(plan))

    def test_prod_promotes_a_dev_verified_digest_and_never_builds(self):
        jobs_ = self.docs["relay-deploy.yml"]["jobs"]
        prod = jobs_["deploy-prod"]
        self.assertEqual(prod["needs"], "plan")
        self.assertNotIn("image_artifact", prod["with"])
        self.assertIn("'prod'", prod["if"])
        self.assertIn("source", prod["with"])
        self.assertTrue(jobs_["deploy-dev"]["with"]["record_verified"])
        self.assertNotIn("record_verified", prod["with"])
        plan = job_text(jobs_["plan"])
        self.assertIn("relay-verified-", plan)
        self.assertIn("prod needs image_digest", plan)
        self.assertIn("az acr import", self.raw["relay-deploy-environment.yml"])

    def test_deploys_do_not_cancel_each_other_and_are_serialised_per_environment(self):
        callee = self.docs["relay-deploy-environment.yml"]["jobs"]["deploy"]
        self.assertIs(callee["concurrency"]["cancel-in-progress"], False)
        self.assertIn("inputs.environment", callee["concurrency"]["group"])
        self.assertTrue(callee["concurrency"]["group"].startswith("relay-deploy-"))

    def test_the_relay_is_deployed_by_digest_never_by_tag(self):
        callee = self.docs["relay-deploy-environment.yml"]["jobs"]["deploy"]
        deploy_steps = [s for s in callee["steps"] if "relayImage=" in str(s.get("run", ""))]
        self.assertEqual([s["name"] for s in deploy_steps], ["Deploy the relay by digest", "Roll back to the previous known-good digest"])
        for step in deploy_steps:
            run = step["run"]
            self.assertEqual(re.findall(r"relayImage=\S+", run), ['relayImage=$RELAY_IMAGE"'], step["name"])
            self.assertRegex(step["env"]["RELAY_IMAGE"], DIGEST_IMAGE)
        self.assertNotIn(":latest", self.raw["relay-deploy-environment.yml"])

    def test_deploy_order_and_rollback(self):
        callee = self.docs["relay-deploy-environment.yml"]["jobs"]["deploy"]
        ids = [s.get("id") for s in callee["steps"] if s.get("id")]
        self.assertEqual(ids, ["target", "infra", "known_good", "image", "deploy", "verify", "record"])
        steps = {s.get("id") or s.get("name") or s["uses"]: s for s in callee["steps"]}
        self.assertIn("deployRelay=false", steps["infra"]["run"])
        self.assertIn("deployRelay=true", steps["deploy"]["run"])
        verify = steps["verify"]["run"]
        for option in ("--relay", "--mode reach", "--mode full", "--json"):
            self.assertIn(option, verify)
        self.assertRegex(verify, r"exit 1")
        rollback = steps["Roll back to the previous known-good digest"]
        condition = " ".join(rollback["if"].split())
        self.assertIn("failure()", condition)
        self.assertIn("steps.deploy.outcome == 'failure'", condition)
        self.assertIn("steps.verify.outcome == 'failure'", condition)
        self.assertIn("relayKnownGoodDigest", steps["known_good"]["run"])
        self.assertIn("relayKnownGoodDigest", steps["record"]["run"])
        position = {name: i for i, name in enumerate(steps)}
        self.assertLess(position["verify"], position["record"])

    def test_image_digest_inputs_are_validated(self):
        self.assertIn("^sha256:[0-9a-f]{64}$", self.raw["relay-deploy-environment.yml"])
        self.assertIn("^sha256:[0-9a-f]{64}$", self.raw["relay-deploy.yml"])

    def test_container_workflow_scans_and_never_logs_in(self):
        doc = self.docs["relay-container.yml"]
        self.assertEqual(set(triggers(doc)), {"workflow_call"})
        text = self.raw["relay-container.yml"]
        self.assertNotIn("azure/login", text)
        self.assertNotIn("id-token", text)
        self.assertIn("hadolint", text)
        self.assertIn("--exit-code 1", text)
        self.assertIn("--severity HIGH,CRITICAL", text)
        self.assertIn("--ignore-unfixed", text)
        self.assertIn("container_smoke.ps1", text)
        built = re.search(r"IMAGE: (\S+)", text).group(1)
        self.assertIn(f"docker tag {built} ", self.raw["relay-deploy-environment.yml"])

    def test_relay_ci_runs_what_the_brief_requires(self):
        text = self.raw["relay-ci.yml"]
        for needle in ("WPIEMU_RELAY_ONLY=ON", "wpiemu_relay_tests", "wpiemu_net_tests", "bicep.tests.ps1", "preflight.tests.ps1",
                       "bootstrap.tests.ps1", "relay-container.yml", "actionlint"):
            self.assertIn(needle, text)

    def test_probe_options_exist_in_the_probe(self):
        probe = (ROOT / "relay" / "src" / "relay_probe.cpp").read_text(encoding="utf-8")
        used = set()
        for run in re.findall(r"wpiemu_relay_probe[^\n]*", self.raw["relay-deploy-environment.yml"]):
            used |= set(re.findall(r"--[a-z-]+", run))
        used |= set(re.findall(r"probe --mode \w+ (--[a-z-]+)", self.raw["relay-deploy-environment.yml"]))
        used |= {"--relay", "--mode", "--timeout-ms", "--count", "--payload-bytes", "--json"}
        for option in used:
            self.assertIn(f'"{option}"', probe, option)
        for mode in ("reach", "full"):
            self.assertIn(f'"{mode}"', probe)

    def test_bicep_contract(self):
        main = (ROOT / "infra" / "relay" / "main.bicep").read_text(encoding="utf-8")
        text = self.raw["relay-deploy-environment.yml"] + self.raw["relay-ci.yml"]
        for parameter in ("deployRelay", "relayImage"):
            self.assertRegex(main, rf"(?m)^param {parameter} ")
        for output in re.findall(r"\.(\w+)\.value", self.raw["relay-deploy-environment.yml"]):
            self.assertRegex(main, rf"(?m)^output {output} ", output)
        names = set(re.findall(r"WPIEMU_RELAY_[A-Z_]+(?=: \$\{\{ vars\.)", text))
        for target in ("dev", "prod"):
            param_file = ROOT / "infra" / "relay" / f"main.{target}.bicepparam"
            read = set(re.findall(r"readEnvironmentVariable\('(\w+)'", param_file.read_text(encoding="utf-8")))
            self.assertEqual(names, read, param_file.name)
        self.assertIn("infra/relay/main.${TARGET}.bicepparam", self.raw["relay-deploy-environment.yml"])


if __name__ == "__main__":
    unittest.main()
