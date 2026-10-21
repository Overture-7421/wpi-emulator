"""Dependency-free release-tool regressions, with no macOS commands or CMake execution."""
import contextlib
import io
import plistlib
import sys
import unittest
from unittest.mock import patch

from support import load, scratch_directory

import package_checksums as CHECKSUMS

MINIMUM_OS = load("check_minimum_os", "packaging/macos/check_minimum_os.py")


class ReleaseToolsTests(unittest.TestCase):
    def setUp(self):
        self.directory = scratch_directory(self, "release-tools-test-")

    def checksums(self, *options):
        with patch.object(sys, "argv", ["package_checksums.py", str(self.directory), *options]):
            with contextlib.redirect_stdout(io.StringIO()):
                CHECKSUMS.main()

    def test_final_artifacts_refresh_and_verify_after_mutation(self):
        artifacts = [self.directory / ("release" + suffix)
                     for suffix in (".exe", ".zip", ".dmg", ".AppImage")]
        for artifact in artifacts:
            artifact.write_bytes(b"pre-signing")
        self.checksums()
        self.checksums("--verify")
        for artifact in artifacts:
            artifact.write_bytes(b"final signed or stapled bytes")
        with self.assertRaisesRegex(SystemExit, "checksum mismatch"):
            self.checksums("--verify")
        self.checksums()
        self.checksums("--verify")
        for artifact in artifacts:
            sidecar = artifact.with_name(artifact.name + ".sha256")
            self.assertEqual(sidecar.read_bytes(),
                             f"{CHECKSUMS.digest(artifact)}  {artifact.name}\n".encode("utf-8"))

    def minimum_os(self, output):
        bundle = self.directory / "wpiemu_app.app"
        contents = bundle / "Contents"
        contents.mkdir(parents=True, exist_ok=True)
        with (contents / "Info.plist").open("wb") as stream:
            plistlib.dump({"LSMinimumSystemVersion": "13.0"}, stream)
        with patch.object(sys, "argv", ["check_minimum_os.py", str(bundle)]):
            with patch.object(MINIMUM_OS.subprocess, "check_output", return_value=output) as command:
                with contextlib.redirect_stdout(io.StringIO()):
                    MINIMUM_OS.main()
                self.assertEqual(command.call_count, 2)
                self.assertEqual(command.call_args.args[0][:4], ["otool", "-arch", "all", "-l"])

    def test_macho_current_and_legacy_minima(self):
        for output in ("cmd LC_BUILD_VERSION\nminos 13.0\n",
                       "cmd LC_VERSION_MIN_MACOSX\ncmdsize 16\nversion 12.0\nsdk 14.0\n"):
            with self.subTest(output=output):
                self.minimum_os(output)

    def test_macho_rejects_newer_slice_or_missing_minimum(self):
        for output in ("minos 13.0\nminos 14.0\n", "no supported minimum command"):
            with self.subTest(output=output):
                with self.assertRaisesRegex(SystemExit, "Mach-O minima"):
                    self.minimum_os(output)


if __name__ == "__main__":
    unittest.main()
