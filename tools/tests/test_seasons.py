"""Season pack checker regressions using a small repository fixture."""
import json
import unittest

from support import REPO, scratch_directory

import check_seasons as SEASONS
import common
import seasons as SEASON_TOOL
from common import write_text

ABI = {"id": "abi_a", "cxxStandard": 20,
       "groups": {key: "g." + key for key in ("hal", "wpiutil", "ntcore", "wpinet")},
       "headers": {"hal": "hal/HAL.h", "wpiutil": "wpi/string.h", "ntcore": "ntcore_c.h"}}


def season(year, **overrides):
    manifest = {
        "year": year, "channel": "stable", "default": year == 3001,
        "wpilib": {"version": f"{year}.1.0", "verified": [f"{year}.1.0"], "installFolder": str(year)},
        "platform": {"abi": "abi_a", "launch": "gradlerio-classic"},
        "toolchain": {"javaMajor": 17},
        "vendors": {"photonvision": {"schema": "photon-us", "defaultVersion": "v1.0", "verified": ["1.0"]}},
        "game": None,
        "samples": [f"samples/s{year}/java"],
    }
    manifest.update(overrides)
    return manifest


class SeasonPackTests(unittest.TestCase):
    def setUp(self):
        self.root = scratch_directory(self, "seasons-test-")
        write_text(self.root / "seasons/abi/abi_a.json", json.dumps(ABI))
        self.add(season(3001))
        self.add(season(3002))

    def add(self, manifest):
        year = manifest["year"]
        write_text(self.root / f"seasons/{year}/season.json", json.dumps(manifest))
        for sample in manifest["samples"]:
            write_text(self.root / sample / "build.gradle",
                       f'plugins {{ id "edu.wpi.first.GradleRIO" version "{manifest["wpilib"]["version"]}" }}\n')

    def errors(self, files=()):
        return SEASONS.check(self.root, list(files))

    def test_a_consistent_set_of_packs_passes(self):
        self.assertEqual(self.errors(), [])

    def test_exactly_one_default_season(self):
        self.add(season(3002, default=True))
        self.assertTrue(any("exactly one season" in error for error in self.errors()))

    def test_the_pinned_version_must_be_verified_and_samples_must_match_it(self):
        self.add(season(3002, wpilib={"version": "3002.1.0", "verified": ["3002.0.0"], "installFolder": "3002"}))
        self.assertTrue(any("must be listed in wpilib.verified" in error for error in self.errors()))
        self.add(season(3002))
        write_text(self.root / "samples/s3002/java/build.gradle",
                   'plugins { id "org.wpilib.GradleRIO" version "3002.0.9" }\n')
        self.assertTrue(any('GradleRIO version is "3002.0.9"' in error for error in self.errors()))

    def test_unknown_abi_and_launch_profile_are_reported(self):
        self.add(season(3002, platform={"abi": "nope", "launch": "gradlerio-future"}))
        errors = "\n".join(self.errors())
        self.assertIn("platform.abi 'nope'", errors)
        self.assertIn("platform.launch must be one of", errors)

    def test_a_sample_must_belong_to_exactly_one_season(self):
        write_text(self.root / "samples/orphan/java/build.gradle", 'plugins { id "edu.wpi.first.GradleRIO" version "x" }\n')
        self.assertTrue(any("samples/orphan/java: not listed" in error for error in self.errors()))

    def test_a_game_plugin_needs_its_directory_and_the_season_id(self):
        self.add(season(3002, game={"plugin": "game_3002"}))
        self.assertTrue(any("needs the directory seasons/3002/game" in error for error in self.errors()))
        write_text(self.root / "seasons/3002/game/CMakeLists.txt", "")
        self.assertEqual(self.errors(), [])
        self.add(season(3002, game={"plugin": "game_other"}))
        self.assertTrue(any("game.plugin must be game_3002" in error for error in self.errors()))

    def test_season_year_literals_are_flagged_in_code_but_not_in_comments_or_product_names(self):
        write_text(self.root / "core/a.cpp", "// 3001 in a comment\nint year = 3001;\n")
        write_text(self.root / "core/b.cpp", 'const char* kVs = "Visual Studio 18 3001";  // name\n')
        write_text(self.root / "core/c.cpp", "int year = 3001;  // season-literal-ok: a documented constant\n")
        write_text(self.root / "tools/x.py", "YEAR = 3002  # the year\n")
        write_text(self.root / "docs/guide.md", "3001 is fine in docs\n")
        write_text(self.root / "tests/t.cpp", "int y = 3001;\n")
        files = ["core/a.cpp", "core/b.cpp", "core/c.cpp", "tools/x.py", "docs/guide.md", "tests/t.cpp"]
        errors = self.errors(files)
        self.assertEqual(sorted(error.split(":")[0] for error in errors), ["core/a.cpp", "tools/x.py"])

    def test_the_allowlist_can_only_shrink(self):
        write_text(self.root / "core/a.cpp", "int a = 3001;\nint b = 3002;\n")
        write_text(self.root / SEASONS.ALLOWLIST, "core/a.cpp = 2\n")
        self.assertEqual(self.errors(["core/a.cpp"]), [])
        write_text(self.root / SEASONS.ALLOWLIST, "core/a.cpp = 3\n")
        self.assertTrue(any("lower its" in error for error in self.errors(["core/a.cpp"])))
        write_text(self.root / SEASONS.ALLOWLIST, "core/a.cpp = 1\n")
        self.assertTrue(any("allowlist permits 1" in error for error in self.errors(["core/a.cpp"])))
        write_text(self.root / "core/a.cpp", "int a = 0;\n")
        self.assertTrue(any("remove the entry" in error for error in self.errors(["core/a.cpp"])))

    def test_the_template_is_not_a_season(self):
        write_text(self.root / "seasons/_template/season.json", json.dumps(season(0)))
        self.assertEqual(list(common.load_seasons(self.root)), ["3001", "3002"])

    def test_workflow_helper_lists_every_extension_in_year_order(self):
        self.assertEqual(SEASON_TOOL.extensions(self.root, "windows"),
                         ["extensions/3001/halsim_wpiemu.dll", "extensions/3002/halsim_wpiemu.dll"])
        self.assertEqual(SEASON_TOOL.extensions(self.root, "macos-bundle")[0],
                         "Contents/PlugIns/wpiemu/3001/libhalsim_wpiemu.dylib")

    def test_the_repository_itself_is_consistent(self):
        self.assertEqual(SEASONS.check(REPO), [])


if __name__ == "__main__":
    unittest.main()
