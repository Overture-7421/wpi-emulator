# Maintaining documentation and the Wiki

Read this when changing documentation, adding a guide, or administering publication.
Prerequisites: a repository checkout, Git and the [pinned Python version](../../versions.json); no native build is needed
for the documentation tooling. Start with the [documentation home](../README.md) and
[contributor policy](../CONTRIBUTING.md#documentation).

## Canonical source and ownership

Repository Markdown is authoritative and reviewed in pull requests. The GitHub Wiki is a generated
mirror, not another editing surface. Change source pages, not generated Wiki pages. Keep contracts
in their [owning references](../README.md), procedures in guides, and user behavior in the
[user guide](../USER_GUIDE.md); link instead of repeating facts.

The exporter owns an explicit mapping in [tools/export_wiki.py](../../tools/export_wiki.py).
`docs/README.md` becomes `Home`; the root README becomes `Project`. Other page names are stable,
flat titles such as `Guide-Motors` and `Reference-Runtime`. Decisions, template/sample READMEs and
packaging guidance are included. Legal material remains linked to its original repository location,
with notices intact. `_Sidebar.md` and `_Footer.md` use the same mapping.

## Update or add a page

1. Choose its canonical home. Add a task guide under `docs/guides` rather than expanding a contract
   into a tutorial. Identify prerequisites, the owner, ordered edits, a source-backed example,
   expected behavior, failure/reset cases, and focused checks.
2. Link it from the [documentation home](../README.md) and the appropriate task index. Keep existing
   useful anchors. Use relative Markdown links to canonical pages and source files.
3. For a new or renamed page, update `PAGES` in the exporter. Pick a unique stable title; do not
   reuse another page's title with different capitalization. New Markdown under `docs`, and
   template/sample/packaging READMEs, cannot silently disappear from the mirror.
4. Verify each snippet in its actual surrounding example. A successful link check is not a compile
   check; a skipped integration test is not evidence of working robot behavior.
   See [building and testing](../BUILDING.md#tests-and-checks) and the owning task guide.
5. From the repository root, run:

```powershell
python tools\check_docs.py
python tools\check_versions.py
python -m unittest discover -s tools\tests -p test_wiki.py -v
python -m unittest discover -s tools\tests -v
python tools\export_wiki.py --output build\wiki-preview --dry-run
python tools\export_wiki.py --output build\wiki-preview
python tools\export_wiki.py --output build\wiki-preview --dry-run
```

The first dry-run validates without creating files; after export, an unchanged repeat reports
`0 changes`. Preview output lives in ignored `build`, never in canonical documentation.
Open the generated Markdown locally or inspect the pull request's `wiki-preview-<SHA>` artifact.
GitHub Wiki rendering itself is not tested by a local Markdown viewer.

Links to mapped pages become Wiki links. Other local files and directories become escaped,
commit-pinned GitHub `blob`/`tree` links; images use a raw media query. Fragments, queries,
reference definitions and titles are retained. Fenced, indented and inline example code is not
rewritten. Missing files, anchors, references, mapping entries and page-name collisions fail export.
The source commit and canonical edit location appear on every page; no timestamps cause churn.

**Preview limitation:** the default revision is checkout `HEAD`. Uncommitted new pages or edits
can be previewed, but their pinned source links will not exist or reflect those edits on GitHub
until committed and pushed. `--revision <full-commit-SHA>` must match checkout `HEAD`; check out the
desired commit first. Publication requires a clean checkout and the current remote `main` revision.

## Administrator setup

Publication is deliberately not attempted from a pull request. The
[documentation workflow](../../.github/workflows/docs.yml) validates and uploads previews without
Wiki secrets. Only trusted pushes to `main` and manual runs selected on `main` can publish.

1. Enable Wiki in the repository settings, then **create and save an initial Wiki page in GitHub**.
   GitHub does not provide the cloneable Wiki repository until its initial page exists.
   A **Repository not found** error from the Wiki remote can mean a disabled/uninitialized Wiki
   or insufficient access; verify both initialization and the publishing account's permissions.
2. The first deployment refuses to overwrite any existing unowned title, including GitHub's initial
   `Home`. Rename that bootstrap page to an unrelated title (for example `Wiki-Bootstrap`) before
   the first publication. Also rename any pre-existing titles that match the mapping, `_Sidebar`
   or `_Footer`. They remain unmanaged; the exporter never adopts human content implicitly.
3. Provision a dedicated publishing account with write access to this repository and a
   **personal access token (classic)**. For this public repository, the minimum scope in this
   supported setup is `public_repo`; private repositories need `repo`. Do not grant `workflow`,
   administrator or deletion scopes. Classic tokens are broader than one repository: minimize the
   account's access, set an expiry, rotate regularly, and authorize organizational SSO if required.
   Organization policy must permit the credential.
4. Create the Actions environment **`wiki`**, restrict deployment branches to `main`, and optionally
   require administrator approval. Add its secret **`WIKI_PUBLISH_TOKEN`**. Never put its value in
   documentation, command arguments, clone URLs, repository variables or generated files.
5. Run **Documentation and Wiki** manually with branch **main**, after these changes merge.
   Missing credentials, inaccessible/uninitialized remotes and unsafe collisions fail explicitly.
   Inspect the job's error and resolve setup rather than treating a skipped deployment as success.

This setup intentionally does not assume that `GITHUB_TOKEN` can write to the separate Wiki Git
repository: GitHub documents its installation token as scoped to the workflow's repository.
The supported publisher uses classic PAT authentication over HTTPS Git, not a REST Wiki endpoint;
fine-grained tokens and GitHub App credentials are not claimed as tested alternatives.
GitHub's authoritative documentation covers
[Wiki initialization and its default branch](https://docs.github.com/en/communities/documenting-your-project-with-wikis/adding-or-editing-wiki-pages),
[token access and handling](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens),
[classic scopes](https://docs.github.com/en/apps/oauth-apps/building-oauth-apps/scopes-for-oauth-apps), and
[`GITHUB_TOKEN` scope](https://docs.github.com/en/actions/concepts/security/github_token).

## Publication, recovery and local publisher tests

Publication regenerates from trusted source rather than downloading a PR artifact. It discovers the
Wiki's default branch (which need not be `main`), clones to a fresh worktree, and applies only files
listed in `.wpiemu-wiki-manifest.json`. The manifest records owned file hashes and source revision.
Unmanaged pages/assets survive. Obsolete owned pages are removed, but hand-edited owned files,
symlinks, malformed manifests and title collisions stop publication before content is applied.

Normal commits include source provenance, `Co-authored-by` and `Copilot-Session` trailers.
Identical output makes no commit or push. Publication is serialized, rejects an outdated source
before cloning and again before pushing, and never force-pushes. A concurrent Wiki edit causes
a normal non-fast-forward failure; no automatic rebase or overwrite hides that conflict.

For recovery, correct canonical source in a pull request and republish current `main`. Restore an
accidentally edited generated page or ownership manifest from Wiki Git history before retrying;
copy any human contribution into canonical source for review first. Never delete the manifest to
work around a collision: that loses ownership and safely causes further collisions, not adoption.
If `main` advances during a run, rerun the workflow on current `main`. Remove a failed local
publication worktree before retrying, or choose a new unused directory.

The regression suite above creates disposable Git repositories under ignored `build`, never calls
the live Wiki, and verifies branch discovery, real no-op/removal commits, unmanaged preservation,
required trailers, stale source rejection and non-fast-forward failure.
For an administrator-controlled **local-only** rehearsal, use local source and Wiki remotes;
the CLI publisher contract is:

```powershell
python tools\export_wiki.py --output build\wiki-preview --publish --wiki-remote build\fixture-wiki.git --wiki-worktree build\wiki-rehearsal --source-remote origin --source-branch main --session local-rehearsal
```

That command requires an initialized disposable Wiki remote and a clean source checkout whose
`origin/main` matches `HEAD`; it is not a setup command and must not be pointed at the live Wiki
for a preview. Remote URLs must not contain credentials. The workflow supplies credentials only
through a short-lived askpass helper, removes the helper on exit and never saves credentials in
Git configuration or artifact content.
