"""Regressions for the helpers shared by the repository tools."""
import contextlib
import io
import unittest
from pathlib import Path

from support import scratch_directory

import common


class CommonTests(unittest.TestCase):
    def setUp(self):
        self.root = scratch_directory(self, "common-test-")

    def test_repository_root_is_the_checkout_that_holds_the_tools(self):
        self.assertEqual(common.REPO_ROOT, Path(__file__).resolve().parents[2])
        self.assertTrue((common.REPO_ROOT / "tools" / "common.py").is_file())
        self.assertTrue((common.REPO_ROOT / "CMakeLists.txt").is_file())

    def test_text_is_utf8_and_strict_unless_asked_to_replace(self):
        path = self.root / "mixed.txt"
        path.write_bytes("caf\u00e9 \u2713\n".encode("utf-8") + b"\xff")
        with self.assertRaises(UnicodeDecodeError):
            common.read_text(path)
        text = common.read_text(path, errors="replace")
        self.assertTrue(text.startswith("caf\u00e9 \u2713\n"))
        self.assertTrue(text.endswith("\ufffd"))

    def test_write_text_creates_parents_and_round_trips(self):
        path = self.root / "a" / "b" / "file.txt"
        common.write_text(path, "caf\u00e9\n")
        self.assertEqual(common.read_text(path), "caf\u00e9\n")

    def test_root_parser_defaults_to_the_repository_and_accepts_an_override(self):
        parser = common.root_parser("description")
        self.assertEqual(parser.parse_args([]).root, common.REPO_ROOT)
        self.assertEqual(parser.parse_args(["--root", "elsewhere"]).root, Path("elsewhere"))

    def test_report_prints_findings_to_the_chosen_stream_with_exit_status_one(self):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out):
            status = common.report(["a: bad", "b: worse"], "ok", heading="Problems:", indent="  ", file=err)
        self.assertEqual(status, 1)
        self.assertEqual(out.getvalue(), "")
        self.assertEqual(err.getvalue(), "Problems:\n  a: bad\n  b: worse\n")

    def test_report_defaults_to_standard_output_and_says_ok_when_clean(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(common.report(["a: bad"], "ok"), 1)
            self.assertEqual(common.report([], "tool: ok"), 0)
        self.assertEqual(out.getvalue(), "a: bad\ntool: ok\n")


if __name__ == "__main__":
    unittest.main()
