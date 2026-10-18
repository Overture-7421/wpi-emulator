"""Regression tests for tools/gen_fiducial_layout.py on synthetic jars (the real jar only when installed)."""
import contextlib
import hashlib
import importlib.util
import io
import json
import os
import shutil
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch


REPO = Path(__file__).resolve().parents[2]


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, REPO / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


GENERATOR = load("gen_fiducial_layout", "tools/gen_fiducial_layout.py")


def tag(ident, x, y, z, w, qx, qy, qz):
    return {"ID": ident,
            "pose": {"translation": {"x": x, "y": y, "z": z},
                     "rotation": {"quaternion": {"W": w, "X": qx, "Y": qy, "Z": qz}}}}


WELDED = {"tags": [tag(2, 1.5, 4.638039999999999, 0.889, 0.6, 0.0, 0.8, -0.0),
                   tag(1, 0, 2, 3, 6.123233995736766e-17, 0.0, 0.0, 1.0)],
          "field": {"length": 16.541, "width": 8.069}}
ANDYMARK = {"tags": [tag(1, 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0)],
            "field": {"length": 16.518, "width": 8.043}}


def with_tag(document, index, **changes):
    document = json.loads(json.dumps(document))
    document["tags"][index].update(changes)
    return document


class GeneratorTests(unittest.TestCase):
    def setUp(self):
        self.directory = REPO / "build" / f"fiducial-layout-test-{os.getpid()}"
        self.directory.mkdir(parents=True, exist_ok=False)
        self.addCleanup(shutil.rmtree, self.directory)
        self.out = self.directory / "fiducial_layouts.cpp"

    def jar(self, welded=WELDED, andymark=ANDYMARK, name="apriltag-java-9999.1.jar", missing=()):
        path = self.directory / name
        with zipfile.ZipFile(path, "w") as archive:
            for spec, document in zip(GENERATOR.LAYOUTS, (welded, andymark)):
                if spec.resource not in missing:
                    archive.writestr(spec.resource, document if isinstance(document, str) else json.dumps(document))
        return path

    def run_main(self, *args):
        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            status = GENERATOR.main(list(args))
        return status, stdout.getvalue(), stderr.getvalue()

    def generate(self, jar):
        status, stdout, stderr = self.run_main("--jar", str(jar), "--out", str(self.out))
        self.assertEqual(status, 0, stderr)
        return self.out.read_bytes().decode("utf-8")

    def check(self, jar):
        return self.run_main("--check", "--jar", str(jar), "--out", str(self.out))

    def test_tables_keep_the_json_text_in_id_order(self):
        text = self.generate(self.jar())
        self.assertIn("    {1, {{0.0f, 2.0f, 3.0f},\n"
                      "         {0.0f, 0.0f, 1.0f, 6.123233995736766e-17f}}, 0.0},\n", text)
        # JSON W, X, Y, Z become x, y, z, w; the sign of a zero is kept
        self.assertIn("    {2, {{1.5f, 4.638039999999999f, 0.889f},\n"
                      "         {0.0f, 0.8f, -0.0f, 0.6f}}, 0.0},\n", text)
        self.assertLess(text.index("{1, {{0.0f, 2.0f"), text.index("{2, {{1.5f"))
        self.assertIn('makeLayout("game_2026.welded", "REBUILT 2026 (welded)",\n'
                      "                                 16.541f, 8.069f, kWeldedTags)", text)
        self.assertLess(text.index("game_2026.welded"), text.index("game_2026.andymark"))  # the first is the default
        self.assertNotIn("\r", text)
        text.encode("ascii")

    def test_provenance_names_the_jar_the_resources_and_the_commands(self):
        text = self.generate(self.jar())
        self.assertIn("apriltag-java:9999.1 (apriltag-java-9999.1.jar)", text)
        for spec, document in zip(GENERATOR.LAYOUTS, (WELDED, ANDYMARK)):
            self.assertIn(spec.resource, text)
            self.assertIn(hashlib.sha256(json.dumps(document).encode("utf-8")).hexdigest(), text)
        self.assertIn("2 tags, field 16.541 x 8.069 m", text)
        self.assertIn("1 tags, field 16.518 x 8.043 m", text)
        self.assertIn("\n//   python tools/gen_fiducial_layout.py\n", text)
        self.assertIn("\n//   python tools/gen_fiducial_layout.py --check\n", text)

    def test_unusable_input_is_refused_with_a_reason(self):
        not_unit = with_tag(WELDED, 0, pose={"translation": {"x": 1, "y": 2, "z": 3},
                                             "rotation": {"quaternion": {"W": 0.5, "X": 0, "Y": 0, "Z": 0}}})
        text = json.dumps(WELDED)
        cases = (
            ("not a unit quaternion", dict(welded=not_unit), "unit length"),
            ("duplicate ids", dict(welded=with_tag(WELDED, 0, ID=1)), "duplicate tag IDs"),
            ("zero id", dict(welded=with_tag(WELDED, 0, ID=0)), "positive integer"),
            ("NaN", dict(welded=text.replace("16.541", "NaN")), "finite"),
            ("number beyond float", dict(welded=text.replace("0.889", "1e39")), "float"),
            ("no tags", dict(welded={"tags": [], "field": WELDED["field"]}), "non-empty"),
            ("no field", dict(welded={"tags": WELDED["tags"]}), "missing 'field'"),
            ("resource missing", dict(missing=(GENERATOR.LAYOUTS[1].resource,)), "not found"),
            ("jar named without a version", dict(name="apriltag.jar"), "apriltag-java-<version>.jar"),
        )
        for label, options, message in cases:
            with self.subTest(label):
                status, _, stderr = self.run_main("--jar", str(self.jar(**options)), "--out", str(self.out))
                self.assertEqual(status, 2)
                self.assertIn(message, stderr)
                self.assertFalse(self.out.exists())
        garbage = self.directory / "apriltag-java-1.jar"
        garbage.write_bytes(b"not a zip")
        status, _, stderr = self.run_main("--jar", str(garbage), "--out", str(self.out))
        self.assertEqual(status, 2)
        self.assertIn("cannot read the jar", stderr)

    def test_check_passes_when_current_and_fails_on_any_difference(self):
        jar = self.jar()
        text = self.generate(jar)
        status, stdout, _ = self.check(jar)
        self.assertEqual(status, 0)
        self.assertIn("up to date", stdout)

        # a Windows checkout with core.autocrlf stores CRLF; that is not a difference
        self.out.write_bytes(text.replace("\n", "\r\n").encode("utf-8"))
        self.assertEqual(self.check(jar)[0], 0)

        self.out.write_bytes(text.replace("0.889f", "0.888f").encode("utf-8"))
        status, _, stderr = self.check(jar)
        self.assertEqual(status, 1)
        self.assertIn("out of date", stderr)
        self.assertIn("python tools/gen_fiducial_layout.py", stderr)
        self.assertIn("0.888f", stderr)

        # the jar changed under an up-to-date file
        self.out.write_bytes(text.encode("utf-8"))
        status, _, stderr = self.check(self.jar(welded=with_tag(WELDED, 0, ID=7)))
        self.assertEqual(status, 1)

        self.out.unlink()
        status, _, stderr = self.check(jar)
        self.assertEqual(status, 1)
        self.assertIn("does not exist", stderr)

    def test_a_missing_default_jar_skips_the_check_but_an_explicit_one_must_exist(self):
        with patch.object(GENERATOR, "default_jar_candidates", return_value=[self.directory / "absent.jar"]):
            status, stdout, _ = self.run_main("--check", "--out", str(self.out))
            self.assertEqual(status, GENERATOR.SKIPPED)
            self.assertIn("SKIPPED", stdout)
            self.assertIn("--jar", stdout)
            status, _, stderr = self.run_main("--out", str(self.out))  # writing the file needs the jar
            self.assertEqual(status, 2)
            self.assertIn("is not installed", stderr)
        status, _, stderr = self.check(self.directory / "missing.jar")
        self.assertEqual(status, 2)
        self.assertIn("no such file", stderr)

    def test_checked_in_source_matches_the_installed_jar(self):
        if not any(path.is_file() for path in GENERATOR.default_jar_candidates()):
            self.skipTest("the WPILib apriltag-java jar is not installed")
        status, _, stderr = self.run_main("--check")
        self.assertEqual(status, 0, stderr)


if __name__ == "__main__":
    unittest.main()
