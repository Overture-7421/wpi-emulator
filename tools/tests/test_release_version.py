"""Release version handling: input validation, tag rules, the CMake rewrite and the installer-name check."""
import contextlib
import io
import os
import sys
import unittest
from unittest.mock import patch

from support import REPO, scratch_directory

import common
import release_version as RV

CMAKE = """cmake_minimum_required(VERSION 3.25)

project(wpi-emulator
    VERSION 0.0.0
    DESCRIPTION "Cross-platform emulator"
    LANGUAGES C CXX)
"""


class ParseTests(unittest.TestCase):
    def test_accepts_with_or_without_v_and_normalizes(self):
        self.assertEqual(RV.parse_release("v0.0.8"), RV.Release("v0.0.8", "0.0.8", False))
        self.assertEqual(RV.parse_release(" 1.12.3 "), RV.Release("v1.12.3", "1.12.3", False))

    def test_suffix_marks_a_prerelease_but_not_the_cmake_version(self):
        self.assertEqual(RV.parse_release("v1.2.0-rc.1"), RV.Release("v1.2.0-rc.1", "1.2.0", True))

    def test_rejects_malformed_versions(self):
        for text in ("", "latest", "v1.2", "v1.2.3.4", "V1.2.3", "v1.2.3+build", "v01.2.3", "v1.2.3-", "v1.2.3 beta"):
            with self.subTest(text=text), self.assertRaises(ValueError):
                RV.parse_release(text)

    def test_rejects_the_development_marker(self):
        for text in ("0.0.0", "v0.0.0-rc1"):
            with self.subTest(text=text), self.assertRaisesRegex(ValueError, "development"):
                RV.parse_release(text)


class TagRuleTests(unittest.TestCase):
    TAGS = ["v0.0.6", "v0.0.7", "v.0.0.1", "alpha.0.0.1", "v0.1.0-rc1"]

    def errors(self, text, tags=None):
        return RV.check_release(RV.parse_release(text), self.TAGS if tags is None else tags)

    def test_next_version_is_fine(self):
        self.assertEqual(self.errors("v0.1.0"), [])
        self.assertEqual(self.errors("v0.1.0-rc2"), [])
        self.assertEqual(self.errors("v0.0.8", []), [])

    def test_existing_tag_is_rejected(self):
        self.assertTrue(any("already exists" in error for error in self.errors("v0.0.7", ["v0.0.7"])))

    def test_lower_than_the_newest_tag_is_rejected(self):
        self.assertTrue(any("lower" in error for error in self.errors("v0.0.5")))

    def test_prerelease_after_its_final_is_rejected(self):
        self.assertTrue(any("prerelease" in error for error in self.errors("v0.0.7-rc1", ["v0.0.7"])))

    def test_unrelated_tags_are_ignored(self):
        self.assertEqual(self.errors("v0.0.1", ["v.0.0.1", "alpha.0.0.1"]), [])


class ApplyTests(unittest.TestCase):
    def test_rewrites_only_the_project_version(self):
        text = RV.set_project_version(CMAKE, "0.0.8")
        self.assertIn("project(wpi-emulator\n    VERSION 0.0.8\n", text)
        self.assertIn("cmake_minimum_required(VERSION 3.25)", text)
        self.assertEqual(RV.read_project_version(text), "0.0.8")
        self.assertEqual(RV.set_project_version(text, "0.0.8"), text)

    def test_needs_exactly_one_project_version(self):
        for text in ("project(x)\n", "project(a VERSION 1.0)\nproject(b VERSION 2.0)\n", ""):
            with self.subTest(text=text), self.assertRaises(ValueError):
                RV.set_project_version(text, "1.2.3")

    def test_real_cmakelists_round_trip(self):
        # The release workflow has already applied its version to this file when it runs CTest: assert no value.
        original = common.read_text(REPO / "CMakeLists.txt")
        current = RV.read_project_version(original)
        changed = RV.set_project_version(original, "9.8.7")
        self.assertEqual(RV.read_project_version(changed), "9.8.7")
        self.assertEqual(RV.set_project_version(changed, current), original)

    def run_cli(self, *argv):
        with patch.object(sys, "argv", ["release_version.py", *argv]):
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                return RV.main()

    def test_cli_apply_and_resolve(self):
        root = scratch_directory(self, "release-version-test-")
        (root / "CMakeLists.txt").write_text(CMAKE, encoding="utf-8")
        self.assertEqual(self.run_cli("--root", str(root), "apply", "v0.0.8-rc1"), 0)
        self.assertEqual(RV.read_project_version(common.read_text(root / "CMakeLists.txt")), "0.0.8")
        self.assertEqual(self.run_cli("--root", str(root), "apply", "nonsense"), 1)

        tags = root / "tags.txt"
        tags.write_text("v0.0.7\nv0.0.6\n", encoding="utf-8")
        output = root / "output.txt"
        with patch.dict(os.environ, {"GITHUB_OUTPUT": str(output)}):
            self.assertEqual(self.run_cli("resolve", "v0.0.8-rc1", "--tags-file", str(tags), "--github-output"), 0)
        self.assertEqual(output.read_text(encoding="utf-8").split(),
                         ["tag=v0.0.8-rc1", "version=0.0.8", "prerelease=true"])
        self.assertEqual(self.run_cli("resolve", "v0.0.7", "--tags-file", str(tags)), 1)


class ArtifactTests(unittest.TestCase):
    def setUp(self):
        self.directory = scratch_directory(self, "release-artifacts-test-")

    def add(self, name, sidecar=True):
        (self.directory / name).write_bytes(b"x")
        if sidecar:
            (self.directory / (name + ".sha256")).write_text("sum\n", encoding="utf-8")

    def test_named_for_the_version_with_sidecars_is_ok(self):
        self.add("wpi-emulator-0.0.8-windows-x64.exe")
        self.add("wpi-emulator-0.0.8-windows-x64.zip")
        self.assertEqual(RV.artifact_errors(self.directory, "0.0.8"), [])

    def test_wrong_version_or_missing_sidecar_is_reported(self):
        self.add("wpi-emulator-0.0.0-linux-x86_64.AppImage")
        self.add("wpi-emulator-0.0.8-macos-arm64-x64.dmg", sidecar=False)
        errors = RV.artifact_errors(self.directory, "0.0.8")
        self.assertEqual(len(errors), 2)

    def test_empty_directory_is_reported(self):
        self.assertEqual(len(RV.artifact_errors(self.directory, "0.0.8")), 1)


if __name__ == "__main__":
    unittest.main()
