"""Deterministic Wiki generation and disposable, local-only Git publication checks."""
import json
import io
import subprocess
import unittest
from pathlib import Path
from unittest.mock import patch

import support
import export_wiki as wiki


SHA = "a" * 40
REPOSITORY = "https://github.com/example/robot"


class WikiFixture(unittest.TestCase):
    def setUp(self):
        self.root = support.scratch_directory(self, "wiki-")
        self.pages = {"docs/README.md": "Home", "docs/topic.md": "Topic"}
        self.write("docs/README.md", "# Welcome\n\n[Topic](topic.md#duplicate-1)\n")
        self.write("docs/topic.md", "# Topic\n\n## Duplicate\n\n## Duplicate\n")

    def write(self, path, text):
        target = self.root / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text, encoding="utf-8")
        return target

    def generate(self, **kwargs):
        return wiki.generate(self.root, SHA, REPOSITORY, self.pages,
                             set(self.pages), **kwargs)


class WikiTests(WikiFixture):
    def test_determinism_navigation_provenance_and_ownership(self):
        files = self.generate()
        self.assertEqual(files, self.generate())
        self.assertIn(b"[Topic](Topic#duplicate-1)", files["Home.md"])
        self.assertIn(SHA.encode(), files["Home.md"])
        self.assertIn(b"blob/" + SHA.encode() + b"/docs/README.md", files["Home.md"])
        self.assertIn(b"[Home](Home)", files["_Sidebar.md"])
        self.assertIn(b"[Editing and publication](Home)", files["_Footer.md"])
        self.assertEqual(wiki.OWNER, json.loads(files[wiki.MANIFEST])["owner"])
        output = self.root / "preview"
        self.assertTrue(wiki.apply(output, files, dry_run=True))
        self.assertFalse(output.exists())
        wiki.apply(output, files)
        self.assertEqual([], wiki.apply(output, files))

    def test_sidebar_groups_link_decisions_index_not_numbered_pages(self):
        for source in wiki.PAGES:
            self.write(source, "# Fixture\n")
        self.write("docs/decisions/README.md",
                   "# Decisions\n\n[Module boundaries](0001-module-boundaries.md)\n")
        files = wiki.generate(self.root, SHA, REPOSITORY, inventory=set(wiki.PAGES))
        sidebar = files["_Sidebar.md"].decode()
        for group in ["Start here", "Extension guides", "References", "Examples", "Maintenance/design"]:
            self.assertIn(f"### {group}\n", sidebar)
        self.assertIn("[Decisions](Decisions)", sidebar)
        self.assertNotIn("Decision-0001", sidebar)
        self.assertIn(b"[Module boundaries](Decision-0001-Module-Boundaries)", files["Decisions.md"])
        linked = {wiki.PAGES["docs/decisions/README.md"]}
        numbered = {page for page in wiki.PAGES.values() if page.startswith("Decision-")}
        for page in wiki.PAGES.values():
            if page in numbered:
                self.assertNotIn(f"]({page})", sidebar)
                self.assertIn(page + ".md", files)
            else:
                self.assertIn(f"]({page})", sidebar)
                linked.add(page)
        self.assertEqual(len(wiki.PAGES) - len(numbered), len(linked))

    def test_local_source_directory_legal_query_encoding_and_image_links(self):
        self.write("src/a b(1).cpp", "int main() {}\n")
        self.write("LICENSE.md", "# License\n")
        self.write("assets/a b.png", "fixture")
        self.write("docs/README.md", """# Home
[source](../src/a%20b%281%29.cpp?plain=1#L1 "title")
[balanced](../src/a\\ b\\(1\\).cpp)
[directory](../src)
[legal](../LICENSE.md#license)
![image](<../assets/a b.png> "alt")
[root](/docs/topic.md?view=1#topic)
[external](https://example.com/a?q=1&x=2#f)
[email](mailto:robot@example.com)
""")
        text = self.generate()["Home.md"].decode()
        self.assertIn(f"blob/{SHA}/src/a%20b%281%29.cpp?plain=1#L1", text)
        self.assertIn(f"tree/{SHA}/src", text)
        self.assertIn(f"blob/{SHA}/LICENSE.md#license", text)
        self.assertIn(f"![image](<{REPOSITORY}/blob/{SHA}/assets/a%20b.png?raw=true>", text)
        self.assertIn("[root](Topic?view=1#topic)", text)
        self.assertIn("[external](https://example.com/a?q=1&x=2#f)", text)
        self.assertIn("[email](mailto:robot@example.com)", text)

    def test_references_html_and_code_are_preserved(self):
        self.write("assets/camera.png", "fixture")
        self.write("docs/README.md", """# Home
[Topic][target] and [target][] and [target]
[target]: topic.md#topic "The title"
![Camera][camera]
[camera]: ../assets/camera.png?size=2#view "Image title"
<a href="topic.md#topic">Topic</a>
`[inline](missing.md)` and ``[more ` code](missing.md)``
```markdown
[fenced](missing.md)
```
~~~markdown
[tilde](missing.md)
~~~~
    [indented](missing.md)
""")
        text = self.generate()["Home.md"].decode()
        self.assertIn('[target]: Topic#topic "The title"', text)
        self.assertIn(f'[camera]: {REPOSITORY}/blob/{SHA}/assets/camera.png?size=2&raw=true#view "Image title"', text)
        self.assertIn('href="Topic#topic"', text)
        for code in ["`[inline](missing.md)`", "``[more ` code](missing.md)``",
                     "[fenced](missing.md)", "[tilde](missing.md)", "[indented](missing.md)"]:
            self.assertIn(code, text)

    def test_undefined_reference_rejected(self):
        self.write("docs/README.md", "[bad][absent]\n")
        with self.assertRaisesRegex(wiki.WikiError, "undefined"):
            self.generate()

    def test_trailing_code_whitespace_is_not_stripped(self):
        self.write("docs/README.md", "# Home\n\n    [code](missing.md)  \n\n")
        self.assertTrue(self.generate()["Home.md"].endswith(b"    [code](missing.md)  \n\n"))

    def test_mapping_collision_missing_omission_and_unsafe_name(self):
        for mapping, inventory, message in [
            ({"docs/README.md": "Topic", "docs/topic.md": "topic"}, set(), "collision"),
            ({"absent.md": "Absent"}, set(), "missing"),
            (self.pages, {"docs/new.md"}, "unmapped"),
            ({"docs/README.md": "../Home"}, set(), "unsafe"),
        ]:
            with self.subTest(message=message), self.assertRaisesRegex(wiki.WikiError, message):
                wiki.generate(self.root, SHA, REPOSITORY, mapping, inventory)

    def test_broken_links_and_anchors_rejected(self):
        for link in ["absent.md", "topic.md#absent", "../../../outside.md",
                     "file:///outside.md", "C:/outside.md"]:
            with self.subTest(link=link):
                self.write("docs/README.md", f"[bad]({link})\n")
                with self.assertRaises(wiki.WikiError):
                    self.generate()

    def test_external_revision_and_repository_validation(self):
        for revision, repository in [("main", REPOSITORY), (SHA, "https://token@github.com/a/b"),
                                     (SHA, "https://not-github.example/a/b")]:
            with self.assertRaises(wiki.WikiError):
                wiki.generate(self.root, revision, repository, self.pages, set(self.pages))

    def test_owned_removal_and_unmanaged_preservation(self):
        output = self.root / "preview"
        files = self.generate()
        wiki.apply(output, files)
        (output / "Personal.md").write_text("# Unmanaged\n", encoding="utf-8")
        (output / "photo.png").write_bytes(b"untouched")
        del self.pages["docs/topic.md"]
        self.write("docs/README.md", "# Home\n")
        self.assertIn("delete Topic.md", wiki.apply(output, self.generate()))
        self.assertFalse((output / "Topic.md").exists())
        self.assertEqual("# Unmanaged\n", (output / "Personal.md").read_text())
        self.assertEqual(b"untouched", (output / "photo.png").read_bytes())

    def test_unmanaged_and_case_and_markup_collisions_preflight(self):
        for index, name in enumerate(["Home.md", "home.md", "Home.markdown", "home.rst"]):
            with self.subTest(name=name):
                output = self.root / f"collision-{index}"
                output.mkdir()
                (output / name).write_text("unmanaged", encoding="utf-8")
                with self.assertRaisesRegex(wiki.WikiError, "collision"):
                    wiki.apply(output, self.generate())
                self.assertEqual([name], [p.name for p in output.iterdir()])

    def test_owned_edits_and_invalid_manifest_are_not_overwritten(self):
        output = self.root / "preview"
        wiki.apply(output, self.generate())
        (output / "Home.md").write_text("human edit", encoding="utf-8")
        with self.assertRaisesRegex(wiki.WikiError, "edited outside"):
            wiki.apply(output, self.generate())
        self.assertEqual("human edit", (output / "Home.md").read_text())
        for manifest in ['{}', '{"owner":"other","files":{}}',
                         json.dumps({"owner": wiki.OWNER, "files": {"../Home.md": "a" * 64}})]:
            (output / wiki.MANIFEST).write_text(manifest, encoding="utf-8")
            with self.assertRaisesRegex(wiki.WikiError, "manifest"):
                wiki.apply(output, self.generate())

    def test_modified_obsolete_file_blocks_all_writes(self):
        output = self.root / "preview"
        wiki.apply(output, self.generate())
        before = (output / "Home.md").read_bytes()
        (output / "Topic.md").write_text("human edit", encoding="utf-8")
        del self.pages["docs/topic.md"]
        self.write("docs/README.md", "# Changed\n")
        with self.assertRaisesRegex(wiki.WikiError, "edited outside"):
            wiki.apply(output, self.generate())
        self.assertEqual(before, (output / "Home.md").read_bytes())
        self.assertEqual("human edit", (output / "Topic.md").read_text())

    def test_manifest_from_another_repository_is_not_adopted(self):
        output = self.root / "preview"
        wiki.apply(output, self.generate())
        files = wiki.generate(self.root, SHA, "https://github.com/other/repository",
                              self.pages, set(self.pages))
        with self.assertRaisesRegex(wiki.WikiError, "manifest"):
            wiki.apply(output, files)

    def test_source_rename_removes_old_owned_page(self):
        output = self.root / "preview"
        wiki.apply(output, self.generate())
        self.pages["docs/topic.md"] = "Renamed"
        changes = wiki.apply(output, self.generate())
        self.assertIn("delete Topic.md", changes)
        self.assertIn("write Renamed.md", changes)
        self.assertIn(b"[Topic](Renamed#duplicate-1)", (output / "Home.md").read_bytes())

    def test_output_and_owned_symlinks_rejected(self):
        output, target = self.root / "preview", self.root / "target"
        target.mkdir()
        try:
            output.symlink_to(target, target_is_directory=True)
        except OSError as error:
            self.skipTest(f"symlinks unavailable: {error}")
        with self.assertRaisesRegex(wiki.WikiError, "symlink"):
            wiki.apply(output, self.generate())
        output.unlink()
        wiki.apply(output, self.generate())
        (output / "Home.md").unlink()
        (output / "Home.md").symlink_to(self.root / "docs" / "README.md")
        with self.assertRaisesRegex(wiki.WikiError, "symlink"):
            wiki.apply(output, self.generate())


class PublisherTests(WikiFixture):
    """Every remote below is a disposable local Git repository, never GitHub."""

    def setUp(self):
        super().setUp()
        self.source = self.root
        self.source_remote = self.root / "build" / "source.git"
        self.wiki_remote = self.root / "build" / "wiki.git"
        self.run_git(self.source, "init", "--quiet", "--initial-branch=main")
        self.run_git(self.source, "config", "user.name", "Fixture")
        self.run_git(self.source, "config", "user.email", "fixture@example.com")
        self.write(".gitignore", "build/\n")
        self.run_git(self.source, "add", ".")
        self.run_git(self.source, "commit", "--quiet", "-m", "source")
        self.source_remote.parent.mkdir(exist_ok=True)
        self.run_git(self.source, "clone", "--quiet", "--bare", str(self.source), str(self.source_remote))
        self.run_git(self.source, "remote", "add", "origin", str(self.source_remote))
        seed = self.root / "build" / "seed"
        seed.mkdir()
        self.run_git(seed, "init", "--quiet", "--initial-branch=wiki-default")
        self.run_git(seed, "config", "user.name", "Fixture")
        self.run_git(seed, "config", "user.email", "fixture@example.com")
        (seed / "Personal.md").write_text("# Personal\n", encoding="utf-8")
        (seed / "photo.png").write_bytes(b"untouched")
        self.run_git(seed, "add", ".")
        self.run_git(seed, "commit", "--quiet", "-m", "unmanaged seed")
        self.run_git(seed, "clone", "--quiet", "--bare", str(seed), str(self.wiki_remote))
        self.revision = wiki.git(self.source, "rev-parse", "HEAD")
        self.counter = 0

    def run_git(self, path, *args):
        location = ["--git-dir", str(path)] if path.suffix == ".git" else ["-C", str(path)]
        result = subprocess.run(["git", *location, *args],
                                capture_output=True, text=True)
        self.assertEqual(0, result.returncode, result.stderr)
        return result.stdout.strip()

    def publish(self):
        self.counter += 1
        files = wiki.generate(self.source, self.revision, REPOSITORY, self.pages, set(self.pages))
        return wiki.publish(self.source, files, self.revision, str(self.wiki_remote),
                            self.root / "build" / f"publish-{self.counter}", session="test-session")

    def test_publication_default_branch_noop_removal_and_trailers(self):
        self.assertIn("Published", self.publish())
        first = self.run_git(self.wiki_remote, "rev-parse", "HEAD")
        message = self.run_git(self.wiki_remote, "log", "-1", "--format=%B")
        self.assertIn("Co-authored-by: Copilot", message)
        self.assertIn("Copilot-Session: test-session", message)
        self.assertIn(self.revision, message)
        self.assertEqual("Wiki already up to date", self.publish())
        self.assertEqual(first, self.run_git(self.wiki_remote, "rev-parse", "HEAD"))
        del self.pages["docs/topic.md"]
        self.write("docs/README.md", "# Home\n")
        (self.source / "docs" / "topic.md").unlink()
        self.run_git(self.source, "add", "-A")
        self.run_git(self.source, "commit", "--quiet", "-m", "remove topic")
        self.run_git(self.source, "push", "--quiet", "origin", "main")
        self.revision = wiki.git(self.source, "rev-parse", "HEAD")
        self.assertIn("Published", self.publish())
        tree = self.run_git(self.wiki_remote, "ls-tree", "--name-only", "HEAD")
        self.assertNotIn("Topic.md", tree)
        self.assertIn("Personal.md", tree)
        self.assertIn("photo.png", tree)
        self.assertEqual("# Personal", self.run_git(self.wiki_remote, "show", "HEAD:Personal.md"))

    def test_stale_source_and_dirty_checkout_cannot_publish(self):
        self.write("untracked.txt", "dirty")
        with self.assertRaisesRegex(wiki.WikiError, "clean"):
            self.publish()
        (self.source / "untracked.txt").unlink()
        self.run_git(self.source_remote, "update-ref", "refs/heads/main", "0" * 40)
        with self.assertRaisesRegex(wiki.WikiError, "missing"):
            self.publish()

    def test_real_source_advance_rejects_stale_checkout(self):
        old_revision = self.revision
        self.write("new.txt", "new source\n")
        self.run_git(self.source, "add", "new.txt")
        self.run_git(self.source, "commit", "--quiet", "-m", "advance source")
        self.run_git(self.source, "push", "--quiet", "origin", "main")
        self.run_git(self.source, "checkout", "--quiet", old_revision)
        with self.assertRaisesRegex(wiki.WikiError, "stale source"):
            self.publish()
        self.assertEqual("unmanaged seed", self.run_git(self.wiki_remote, "log", "-1", "--format=%s"))

    def test_git_inventory_omission_and_cli_dry_run(self):
        self.write("docs/new.md", "# New\n")
        self.assertIn("docs/new.md", wiki.source_inventory(self.source))
        with self.assertRaisesRegex(wiki.WikiError, "unmapped"):
            wiki.generate(self.source, self.revision, REPOSITORY, self.pages)
        (self.source / "docs" / "new.md").unlink()
        output = self.root / "build" / "preview"
        arguments = ["export_wiki.py", "--root", str(self.source), "--output", str(output),
                     "--repository", REPOSITORY, "--dry-run"]
        with patch.object(wiki, "PAGES", self.pages), patch("sys.argv", arguments), \
                patch("sys.stdout", new_callable=io.StringIO):
            self.assertEqual(0, wiki.main())
        self.assertFalse(output.exists())
        arguments[-1:] = ["--revision", "b" * 40]
        with patch.object(wiki, "PAGES", self.pages), patch("sys.argv", arguments), \
                patch("sys.stderr", new_callable=io.StringIO):
            self.assertEqual(1, wiki.main())
        self.assertFalse(output.exists())

    def test_source_advance_before_push_is_rejected(self):
        with patch.object(wiki, "remote_head", side_effect=[self.revision, "b" * 40]):
            with self.assertRaisesRegex(wiki.WikiError, "advanced"):
                self.publish()
        self.assertEqual("unmanaged seed", self.run_git(self.wiki_remote, "log", "-1", "--format=%s"))

    def test_uninitialized_wiki_and_embedded_credentials_fail(self):
        empty = self.root / "build" / "empty.git"
        self.run_git(self.source, "init", "--quiet", "--bare", str(empty))
        self.wiki_remote = empty
        with self.assertRaisesRegex(wiki.WikiError, "initial page"):
            self.publish()
        self.wiki_remote = "https://user:secret@github.com/example/repo.wiki.git"
        with self.assertRaisesRegex(wiki.WikiError, "credentials"):
            self.publish()

    def test_non_fast_forward_push_preserves_remote(self):
        real_git = wiki.git
        changed = False

        def racing_git(root, *args):
            nonlocal changed
            if args[0] == "push" and not changed:
                changed = True
                seed = self.root / "build" / "seed"
                (seed / "Other.md").write_text("other writer\n", encoding="utf-8")
                self.run_git(seed, "add", ".")
                self.run_git(seed, "commit", "--quiet", "-m", "concurrent writer")
                self.run_git(seed, "push", "--quiet", str(self.wiki_remote), "wiki-default")
            return real_git(root, *args)

        with patch.object(wiki, "git", side_effect=racing_git):
            with self.assertRaisesRegex(wiki.WikiError, "push failed"):
                self.publish()
        self.assertEqual("concurrent writer", self.run_git(self.wiki_remote, "log", "-1", "--format=%s"))


if __name__ == "__main__":
    unittest.main()
