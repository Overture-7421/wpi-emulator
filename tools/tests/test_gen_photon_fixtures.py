"""Regression tests for tools/gen_photon_fixtures.py: its pure parts, a faked Java run, and (only when the JDK and the
PhotonLib jars are installed) the real library against the committed fixtures."""
import contextlib
import io
import importlib.util
import math
import os
import shutil
import unittest
from pathlib import Path
from unittest.mock import patch


REPO = Path(__file__).resolve().parents[2]


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, REPO / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


GENERATOR = load("gen_photon_fixtures", "tools/gen_photon_fixtures.py")

UUID = "4b2ff16a964b5e2bf04be0c1454d91c4"


def fake_answer(cases):
    schemas = (GENERATOR.Schema("PhotonPipelineResult", UUID, "PhotonPipelineMetadata:aa metadata;"),
               GENERATOR.Schema("TargetCorner", "16f6ac0dedc8eaccb951f4895d9e18b6", "float64 x;float64 y;"))
    return GENERATOR.LibraryAnswer("v9.9.9", 127, UUID, schemas, {case.name: "00ff" for case in cases})


def read_numbers(words, index, count):
    """`count` words from `index`, each of which must be a number; returns the next index."""
    for word in words[index:index + count]:
        float(word)
    if index + count > len(words):
        raise AssertionError("the line ends early")
    return index + count


def check_line_shape(test, line):
    """The grammar documented in the generator, parsed independently of the generator's own writers."""
    words = line.split()
    kind = words[0]
    if kind == "message":
        test.assertEqual(len(words), 2)
    elif kind == "meta":
        test.assertEqual(len(words), 5)
        for word in words[1:]:
            int(word)
    elif kind == "value":
        test.assertEqual(len(words), 8)
        read_numbers(words, 1, 7)
    elif kind in ("target", "multi"):
        i = read_numbers(words, 1, 8 if kind == "target" else 3)
        for keyword in ("best", "alt"):
            test.assertEqual(words[i], keyword)
            i = read_numbers(words, i + 1, 7)
        if kind == "target":
            for keyword in ("rect", "det"):
                test.assertEqual(words[i], keyword)
                count = int(words[i + 1])
                i = read_numbers(words, i + 2, 2 * count)
        else:
            test.assertEqual(words[i], "ids")
            count = int(words[i + 1])
            for word in words[i + 2:i + 2 + count]:
                int(word)
            i += 2 + count
        test.assertEqual(i, len(words), "unexpected words at the end of: " + line[:80])
    else:
        test.fail("unknown fixture line: " + line[:80])


class GeneratorTests(unittest.TestCase):
    def setUp(self):
        self.directory = REPO / "build" / f"photon-fixtures-test-{os.getpid()}"
        self.directory.mkdir(parents=True, exist_ok=False)
        self.addCleanup(shutil.rmtree, self.directory)
        self.out = self.directory / "photon"

    def run_main(self, *args, generated=None):
        """main() with the Java run replaced by `generated` = (uuid, files), or with a missing toolchain."""
        stdout, stderr = io.StringIO(), io.StringIO()
        if generated is None:
            patcher = patch.object(GENERATOR, "find_java", return_value=None)
        else:
            patcher = patch.object(GENERATOR, "generate", return_value=generated)
        with patcher, contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            status = GENERATOR.main(["--out", str(self.out), *args])
        return status, stdout.getvalue(), stderr.getvalue()

    def generated(self):
        cases = GENERATOR.build_cases()
        return UUID, GENERATOR.render_files(cases, fake_answer(cases))

    # ---- pure parts ------------------------------------------------------------------------------------------

    def test_numbers_are_the_shortest_text_that_parses_back(self):
        self.assertEqual(GENERATOR.num(0.1), "0.1")
        self.assertEqual(GENERATOR.num(-0.0), "-0.0")
        self.assertEqual(GENERATOR.num(5), "5")
        self.assertEqual(GENERATOR.num(-9223372036854775808), "-9223372036854775808")
        for value in (1 / 3, 1e300, 5e-324, 1.7976931348623157e308, 123456789.123456789, -2.5e-7):
            self.assertEqual(float(GENERATOR.num(value)), value)
        for bad in (math.nan, math.inf, -math.inf):
            with self.assertRaises(GENERATOR.GenerationError):
                GENERATOR.num(bad)
        with self.assertRaises(TypeError):
            GENERATOR.num(True)

    def test_unit_quaternions_survive_the_normalisation_photonlib_applies(self):
        # Quaternion.normalize(): divide every component by sqrt(w*w + x*x + y*y + z*z) unless that is 0.
        near_unit = ((0.9659258262890683, 0.0, 0.25881904510252074, 0.0), (0.984807753012208, 0.0, -0.17364817766693033, 0.0),
                     (0.5, 0.5, 0.5, 0.5), (0.0, 0.0, 0.0, 1.0), (0.7071067811865476, 0.7071067811865476, 0.0, 0.0))
        for raw in near_unit + ((0.3, -0.4, 0.5, 0.2),):
            w, x, y, z = GENERATOR.unit_quaternion(*raw)
            norm = math.sqrt(w * w + x * x + y * y + z * z)
            self.assertEqual(norm, 1.0, raw)
            self.assertEqual((w / norm, x / norm, y / norm, z / norm), (w, x, y, z), raw)
        for raw in near_unit:
            moved = max(abs(a - b) for a, b in zip(GENERATOR.unit_quaternion(*raw), raw))
            self.assertLess(moved, 1e-15, raw)  # an almost-unit input only moves in the last bits

    def test_the_cases_have_unique_names_and_follow_the_documented_line_grammar(self):
        cases = GENERATOR.build_cases()
        names = [case.name for case in cases]
        self.assertEqual(len(set(names)), len(names))
        for case in cases:
            with self.subTest(case.name):
                self.assertRegex(case.name, r"^[a-z0-9_]+$")
                self.assertTrue(case.description)
                self.assertIn(case.lines[0], ("message PhotonPipelineResult", "message Transform3d"))
                for line in case.lines:
                    check_line_shape(self, line)
        by_name = {case.name: case for case in cases}
        # The lists the cases promise, counted from the text.
        self.assertEqual(sum(line.startswith("target ") for line in by_name["max_targets"].lines), GENERATOR.MAX_LIST)
        self.assertIn(f"rect {GENERATOR.MAX_LIST} ", by_name["max_corners"].lines[2])
        self.assertIn(f" det {GENERATOR.MAX_LIST} ", by_name["max_corners"].lines[2])
        self.assertFalse(any(line.startswith(("target", "multi")) for line in by_name["empty"].lines))

    def test_the_java_output_is_parsed_and_anything_else_is_refused(self):
        cases = GENERATOR.build_cases()[:2]
        text = ("version\tv9.9.9\nmaxlist\t127\n"
                f"schema\tPhotonPipelineResult\t{UUID}\ttext one;\n"
                "schema\tTargetCorner\t16f6ac0dedc8eaccb951f4895d9e18b6\tfloat64 x;float64 y;\n"
                f"bytes\t{cases[0].name}\t0102\nbytes\t{cases[1].name}\taabb\n")
        answer = GENERATOR.parse_java_output(text, [case.name for case in cases])
        self.assertEqual((answer.version, answer.max_list, answer.root_uuid), ("v9.9.9", 127, UUID))
        self.assertEqual(answer.bytes_by_case[cases[1].name], "aabb")
        self.assertEqual([schema.name for schema in answer.schemas], ["PhotonPipelineResult", "TargetCorner"])
        for broken, message in ((text + "surprise\n", "unexpected output"),
                                (text.replace("version\tv9.9.9\n", ""), "did not report"),
                                (text.replace(f"bytes\t{cases[1].name}\taabb\n", ""), "no bytes for")):
            with self.assertRaises(GENERATOR.GenerationError) as raised:
                GENERATOR.parse_java_output(broken, [case.name for case in cases])
            self.assertIn(message, str(raised.exception))

    def test_the_java_input_is_one_block_per_case(self):
        cases = GENERATOR.build_cases()
        text = GENERATOR.java_input(cases)
        self.assertEqual(text.count("fixture "), len(cases))
        self.assertEqual(text.splitlines().count("end"), len(cases))
        text.encode("ascii")

    def test_the_rendered_files_are_ascii_lf_and_say_where_they_came_from(self):
        _, files = self.generated()
        self.assertIn("schemas.txt", files)
        self.assertEqual(len(files), len(GENERATOR.build_cases()) + 1)
        for name, text in files.items():
            with self.subTest(name):
                text.encode("ascii")
                self.assertNotIn("\r", text)
                self.assertTrue(text.endswith("\n"))
                self.assertIn("Generated by tools/gen_photon_fixtures.py", text)
        self.assertIn("\nbytes 00ff\n", files["empty.fixture"])
        self.assertIn(f"\nPhotonPipelineResult\t{UUID}\t", files["schemas.txt"])
        self.assertIn("v9.9.9", files["schemas.txt"])
        self.assertEqual(files, self.generated()[1])  # deterministic

    # ---- the command line ------------------------------------------------------------------------------------

    def test_write_then_check_passes_and_any_difference_fails(self):
        status, stdout, stderr = self.run_main(generated=self.generated())
        self.assertEqual(status, 0, stderr)
        directory = self.out / UUID
        self.assertTrue((directory / "schemas.txt").is_file())
        self.assertEqual(len(list(directory.glob("*.fixture"))), len(GENERATOR.build_cases()))

        status, stdout, _ = self.run_main("--check", generated=self.generated())
        self.assertEqual(status, 0)
        self.assertIn("up to date", stdout)

        # a Windows checkout with core.autocrlf stores CRLF; that is not a difference
        for path in directory.iterdir():
            path.write_bytes(path.read_bytes().replace(b"\n", b"\r\n"))
        self.assertEqual(self.run_main("--check", generated=self.generated())[0], 0)

        (directory / "empty.fixture").write_bytes(b"# changed\n")
        status, _, stderr = self.run_main("--check", generated=self.generated())
        self.assertEqual(status, 1)
        self.assertIn("empty.fixture is out of date", stderr)
        self.assertIn("python tools/gen_photon_fixtures.py", stderr)

        (directory / "empty.fixture").unlink()
        (directory / "left_over.fixture").write_bytes(b"x\n")
        status, _, stderr = self.run_main("--check", generated=self.generated())
        self.assertEqual(status, 1)
        self.assertIn("empty.fixture does not exist", stderr)
        self.assertIn("left_over.fixture is not produced any more", stderr)

    def test_writing_removes_fixtures_that_are_not_produced_any_more(self):
        directory = self.out / UUID
        directory.mkdir(parents=True)
        (directory / "renamed_case.fixture").write_bytes(b"old\n")
        (directory / "notes.txt").write_bytes(b"not a fixture\n")
        self.assertEqual(self.run_main(generated=self.generated())[0], 0)
        self.assertFalse((directory / "renamed_case.fixture").exists())
        self.assertTrue((directory / "notes.txt").exists())  # only *.fixture files are managed

    def test_a_missing_toolchain_skips_the_check_but_the_write_must_have_one(self):
        status, stdout, _ = self.run_main("--check")
        self.assertEqual(status, GENERATOR.SKIPPED)
        self.assertIn("SKIPPED", stdout)
        status, _, stderr = self.run_main()
        self.assertEqual(status, 2)
        self.assertIn("no JDK found", stderr)
        self.assertFalse(self.out.exists())

    def test_an_unusable_answer_from_the_library_is_an_error(self):
        with patch.object(GENERATOR, "generate", side_effect=GENERATOR.GenerationError("the Java program failed")):
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                self.assertEqual(GENERATOR.main(["--out", str(self.out), "--check"]), 2)
            self.assertIn("the Java program failed", stderr.getvalue())

    def test_jars_are_found_in_the_maven_and_in_the_gradle_layout(self):
        maven = self.directory / "maven" / "wpimath-java" / "2026.2.1"
        maven.mkdir(parents=True)
        (maven / "wpimath-java-2026.2.1.jar").write_bytes(b"")
        (maven / "wpimath-java-2026.2.1-sources.jar").write_bytes(b"")
        other = self.directory / "maven" / "wpimath-java" / "2025.9.9"
        other.mkdir(parents=True)
        (other / "wpimath-java-2025.9.9.jar").write_bytes(b"")
        gradle = self.directory / "gradle" / "photontargeting-java" / "v2026.3.4" / "0123abcd"
        gradle.mkdir(parents=True)
        (gradle / "photontargeting-java-v2026.3.4.jar").write_bytes(b"")

        found = GENERATOR.find_jar([self.directory / "maven" / "wpimath-java"], "wpimath-java", None)
        self.assertEqual(found.name, "wpimath-java-2026.2.1.jar")  # the highest version, never -sources
        found = GENERATOR.find_jar([self.directory / "maven" / "wpimath-java"], "wpimath-java", "2025.9.9")
        self.assertEqual(found.name, "wpimath-java-2025.9.9.jar")
        found = GENERATOR.find_jar([self.directory / "gradle" / "photontargeting-java"], "photontargeting-java", "v2026.3.4")
        self.assertEqual(found.parent.name, "0123abcd")
        self.assertIsNone(GENERATOR.find_jar([self.directory / "maven" / "wpimath-java"], "wpimath-java", "1.0"))
        self.assertIsNone(GENERATOR.find_jar([self.directory / "absent"], "wpimath-java", None))

    def test_the_default_release_is_the_samples_vendordep(self):
        vendordep = GENERATOR.read_vendordep()
        self.assertRegex(vendordep["version"], r"^v\d{4}\.\d+\.\d+$")
        self.assertRegex(vendordep["year"], r"^\d{4}$")
        self.assertTrue(vendordep["urls"])
        self.assertTrue(GENERATOR.JAVA_PROGRAM.is_file())

    # ---- the real library ------------------------------------------------------------------------------------

    def test_the_committed_fixtures_match_what_the_installed_photonlib_produces(self):
        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            status = GENERATOR.main(["--check"])
        if status == GENERATOR.SKIPPED:
            self.skipTest("the JDK or the PhotonLib jars are not installed")
        self.assertEqual(status, 0, stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
