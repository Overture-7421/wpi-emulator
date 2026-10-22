"""Version-pin checker regressions using a small repository fixture."""
import unittest

from support import scratch_directory

import check_versions as VERSIONS
from common import load_versions, write_text

PINS = [
    VERSIONS.Pin("wpilib", ("ci.yml", "samples/*/build.gradle"), r'version\s+"([^"]+)"', "plugin version"),
    VERSIONS.Pin("qt.release", ("build.sh",), r"Qt/(\d+\.\d+\.\d+)/", "Qt path"),
    VERSIONS.Pin("cmake", ("presets.json",), r'"major":\s*(\d+),\s*"minor":\s*(\d+)', "minimum CMake"),
]


class VersionPinTests(unittest.TestCase):
    def setUp(self):
        self.root = scratch_directory(self, "versions-test-")
        write_text(self.root / "versions.json", '{"wpilib": "2026.2.1", "qt": {"release": "6.8.3"}, "cmake": "3.25"}')
        write_text(self.root / "ci.yml", 'plugin version "2026.2.1"\n')
        write_text(self.root / "samples/a/build.gradle", 'plugin version "2026.2.1"\n')
        write_text(self.root / "build.sh", "QT_ROOT_DIR=$HOME/Qt/6.8.3/gcc_64\n")
        write_text(self.root / "presets.json", '{ "major": 3, "minor": 25, "patch": 0 }\n')

    def check(self):
        return VERSIONS.check(self.root, load_versions(self.root), PINS)

    def test_copies_that_agree_pass(self):
        self.assertEqual(self.check(), [])

    def test_mismatch_names_file_line_and_both_values(self):
        write_text(self.root / "ci.yml", '# header\nplugin version "2026.2.1"\n\nplugin version "2026.2.0"\n')
        write_text(self.root / "samples/a/build.gradle", 'plugin version "2027.1.1"\n')
        self.assertEqual(self.check(), [
            'ci.yml:4: plugin version is "2026.2.0", but versions.json wpilib is "2026.2.1"',
            'samples/a/build.gradle:1: plugin version is "2027.1.1", but versions.json wpilib is "2026.2.1"'])

    def test_groups_of_one_match_form_a_single_version(self):
        write_text(self.root / "presets.json", '{ "major": 3, "minor": 24 }\n')
        self.assertEqual(self.check(),
                         ['presets.json:1: minimum CMake is "3.24", but versions.json cmake is "3.25"'])

    def test_jdk_assets_must_be_self_consistent(self):
        asset = {"version": "17.0.20.1", "size": 190817615, "sha256": "e" * 64,
                 "url": "https://github.com/adoptium/x/download/jdk-17.0.20.1%2B1/OpenJDK17U_17.0.20.1_1.zip"}
        good = {"jdk": {"seasons": {"2026": 17}, "assets": {"17-windows-x64": asset}}}
        self.assertEqual(VERSIONS.check_jdk_assets(good), [])
        bad = {"jdk": {"seasons": {"2026": 17}, "assets": {"17-windows-x64": {
            "version": "17.0.9", "size": 900 * 1024 * 1024, "sha256": "XYZ", "url": "http://example.com/jdk.zip"}}}}
        self.assertEqual(len(VERSIONS.check_jdk_assets(bad)), 4)
        self.assertEqual(len(VERSIONS.check_jdk_assets({})), 1)

    def test_every_season_needs_a_download_of_its_java_for_every_platform(self):
        asset = {"version": "17.0.20.1", "size": 1, "sha256": "e" * 64,
                 "url": "https://github.com/adoptium/x/jdk-17.0.20.1/OpenJDK17U_17.0.20.1_1.zip"}
        pins = {"jdk": {"seasons": {"2026": 17, "2027": 25}, "assets": {"17-windows-x64": asset, "17-linux-x64": asset}}}
        errors = VERSIONS.check_jdk_assets(pins)
        self.assertEqual(len(errors), 2)
        self.assertIn("jdk.seasons.2027: Java 25 has no download for linux-x64", errors[0])
        wrong_key = {"jdk": {"seasons": {"2026": 17}, "assets": {"windows-x64": asset}}}
        self.assertIn("<java major>-<os>-<arch>", VERSIONS.check_jdk_assets(wrong_key)[0])

    def test_missing_file_missing_pin_and_unknown_key_are_distinct_errors(self):
        (self.root / "build.sh").unlink()
        write_text(self.root / "presets.json", "{}\n")
        errors = VERSIONS.check(self.root, load_versions(self.root),
                                PINS + [VERSIONS.Pin("absent", ("ci.yml",), r"(.)", "unlisted pin")])
        self.assertEqual(len(errors), 3)
        self.assertIn("build.sh: no such file (checked as Qt path)", errors[0])
        self.assertIn("presets.json: no minimum CMake found", errors[1])
        self.assertEqual(errors[2], "versions.json: no entry 'absent' (needed to check unlisted pin)")


if __name__ == "__main__":
    unittest.main()
