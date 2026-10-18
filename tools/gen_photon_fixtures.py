#!/usr/bin/env python3
"""Generate the golden PhotonVision fixtures of the vision bridge from the REAL PhotonLib serde.

The bridge writes PhotonVision's result message (photonstruct PhotonPipelineResult, ADR 0017) with its own generic
encoder. This tool is how that encoder is proven against the real thing: it packs a handful of fixed inputs with
PhotonLib's own serde (photontargeting-java, run by tools/gen_photon_fixtures.java in the WPILib JDK) and commits the
bytes next to the inputs. tests/bridges_vision/test_photon_*.cpp then encode the SAME inputs with the bridge's encoder
and require byte equality, and compare the message schema texts of the bridge's profile with the ones the real
library reports. No byte of a fixture is ever typed or copied by hand.

    python tools/gen_photon_fixtures.py                       # rewrite tests/bridges_vision/fixtures/photon/<uuid>/
    python tools/gen_photon_fixtures.py --check               # fail when the committed fixtures are out of date
    python tools/gen_photon_fixtures.py --photonlib-version v2026.1.1     # another PhotonLib release
    python tools/gen_photon_fixtures.py --download            # fetch missing PhotonLib jars from the vendordep's Maven

What it needs (nothing is installed or modified): a JDK 17+ (JAVA_HOME, the WPILib JDK at %PUBLIC%/wpilib/<year>/jdk or
~/wpilib/<year>/jdk, or java on PATH; --java), the jars photontargeting-java and photonlib-java of the release (the
Gradle cache once samples/vision_robot/java has been built; --photon-jar; or --download into build/photon-fixture-jars)
and the WPILib Java libraries (wpimath, wpiutil, ...) of the WPILib install (%PUBLIC%/wpilib/<year>/maven or
~/wpilib/<year>/maven; --wpilib-maven; or the Gradle cache). The default release is the one in
samples/vision_robot/java/vendordeps/photonlib.json. Exit status: 0 done / up to date, 1 --check found a difference,
2 unusable input or a failure, 77 --check was skipped because the JDK or the jars are not installed (the CTest "skip"
convention). Standard library only.

Layout: <out>/<message UUID>/schemas.txt (the message definitions the library reports, one "name<TAB>uuid<TAB>text"
line each, plus the PhotonLib version and the list limit) and one <case>.fixture per case. A fixture is a text file;
lines starting with '#' are comments, numbers are the shortest text that parses back to the same double:

    message PhotonPipelineResult                 (or "message Transform3d": the payload of a struct:Transform3d topic)
    meta <sequenceID> <captureTimestampMicros> <publishTimestampMicros> <timeSinceLastPong>
    target <yaw> <pitch> <area> <skew> <fiducialId> <objDetectId> <objDetectConf> <poseAmbiguity>
           best <x y z qw qx qy qz> alt <x y z qw qx qy qz> rect <n> <x y>... det <n> <x y>...     (ONE line)
    multi <bestReprojErr> <altReprojErr> <ambiguity> best <7 numbers> alt <7 numbers> ids <n> <id>...
    value <x y z qw qx qy qz>                    (Transform3d only)
    bytes <hex>                                  (written by this tool: what the real serde produced)

PhotonLib normalises every quaternion it packs, so the inputs use quaternions that are exactly unit length in double
arithmetic (unit_quaternion below); the Java side refuses an input that would not survive the real serde bit for bit.

Adding a PhotonLib release (see also the header of bridges/vision/include/wpiemu/bridges/vision/photon_profile.h):
  1. Same message UUID (schemas.txt unchanged): run with --photonlib-version <vX.Y.Z> and check the fixtures do not
     change, then extend the version rule in sdk/include/wpiemu/sdk/vision/photon_profiles.h.
  2. New UUID: run it (a new <uuid>/ directory appears; if the PhotonLib constructors changed, adapt
     tools/gen_photon_fixtures.java), copy the schema texts of schemas.txt into
     bridges/vision/src/photon/photon_profile_data.cpp as a new profile, add a mapper field in photon_mapper.cpp for any
     new schema field, extend photonMessageUuidForVersion, and run the vision tests. A profile without fixtures, with
     texts that differ from schemas.txt, or with a field the mapper cannot fill, fails them (or is refused at run time).
"""
from __future__ import annotations

import argparse
import difflib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import NamedTuple

REPO = Path(__file__).resolve().parents[1]
VENDORDEP = REPO / "samples" / "vision_robot" / "java" / "vendordeps" / "photonlib.json"
SAMPLE_GRADLE = REPO / "samples" / "vision_robot" / "java" / "build.gradle"
DEFAULT_OUT = REPO / "tests" / "bridges_vision" / "fixtures" / "photon"
DOWNLOAD_DIR = REPO / "build" / "photon-fixture-jars"
JAVA_PROGRAM = Path(__file__).with_suffix(".java")
SKIPPED = 77
DIFF_LINES = 40
MAX_LIST = 127  # PhotonLib's Packet.MAX_ARRAY_LEN

# The two PhotonLib artifacts (group org.photonvision) the program needs, and the libraries below them. WPILib's own
# jars carry the WPILib version, the rest is third-party. "required" ones must be found.
PHOTON_ARTIFACTS = ("photontargeting-java", "photonlib-java")
LIBRARIES = (
    ("edu.wpi.first.wpimath", "wpimath-java", True, True),
    ("edu.wpi.first.wpiutil", "wpiutil-java", True, True),
    ("edu.wpi.first.wpiunits", "wpiunits-java", True, False),
    ("us.hebi.quickbuf", "quickbuf-runtime", False, False),
    ("com.fasterxml.jackson.core", "jackson-annotations", False, False),
    ("com.fasterxml.jackson.core", "jackson-core", False, False),
    ("com.fasterxml.jackson.core", "jackson-databind", False, False),
    ("org.ejml", "ejml-core", False, False),
    ("org.ejml", "ejml-ddense", False, False),
    ("org.ejml", "ejml-dsparse", False, False),
    ("org.ejml", "ejml-simple", False, False),
    ("org.ejml", "ejml-fdense", False, False),
    ("org.ejml", "ejml-fsparse", False, False),
    ("org.ejml", "ejml-cdense", False, False),
    ("org.ejml", "ejml-zdense", False, False),
)  # (group, artifact, required, carries the WPILib version)


class GenerationError(Exception):
    """The inputs, the toolchain or the real library's answer are unusable."""


class ToolsMissing(GenerationError):
    """The JDK or a jar is not installed: --check is skipped instead of failing."""


# ---- the cases -------------------------------------------------------------------------------------------------

class Case(NamedTuple):
    name: str
    description: str
    lines: tuple  # the fixture input, without the bytes line


def num(value) -> str:
    """The shortest text that parses back to the same double (integers stay integers)."""
    if isinstance(value, bool):
        raise TypeError("bool is not a number")
    if isinstance(value, int):
        return str(value)
    value = float(value)
    if not math.isfinite(value):
        raise GenerationError(f"{value} cannot be written in a fixture")
    return repr(value)


def unit_quaternion(w: float, x: float, y: float, z: float) -> tuple:
    """A quaternion whose norm is EXACTLY 1.0 in double arithmetic, close to the one given.

    PhotonLib normalises every rotation it packs (Rotation3d(Quaternion) divides by sqrt(w*w + x*x + y*y + z*z)),
    so an input only survives the real serde bit for bit when that division is a no-op. Normalising again and
    again reaches such a fixed point within a couple of steps; the operations are those of Quaternion.normalize()."""
    for _ in range(16):
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm == 1.0:
            return (w, x, y, z)
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
    raise GenerationError("no exactly unit quaternion near " + repr((w, x, y, z)))


def transform(x, y, z, quaternion) -> list:
    return [x, y, z, *unit_quaternion(*quaternion)]


def corners(points) -> list:
    return [len(points)] + [value for point in points for value in point]


def target_line(yaw, pitch, area, skew, fiducial_id, obj_id, conf, ambiguity, best, alt, rect, det) -> str:
    words = ["target", num(yaw), num(pitch), num(area), num(skew), num(fiducial_id), num(obj_id), num(conf),
             num(ambiguity), "best", *map(num, best), "alt", *map(num, alt),
             "rect", *map(num, corners(rect)), "det", *map(num, corners(det))]
    return " ".join(words)


def multi_line(best_error, alt_error, ambiguity, best, alt, ids) -> str:
    words = ["multi", num(best_error), num(alt_error), num(ambiguity), "best", *map(num, best), "alt", *map(num, alt),
             "ids", num(len(ids)), *map(num, ids)]
    return " ".join(words)


def meta_line(sequence, capture, publish, pong) -> str:
    return f"meta {num(sequence)} {num(capture)} {num(publish)} {num(pong)}"


# Rotations about Y by a few angles, as decimal text (cos and sin of the half angle).
PITCH_30 = (0.9659258262890683, 0.0, 0.25881904510252074, 0.0)
PITCH_MINUS_20 = (0.984807753012208, 0.0, -0.17364817766693033, 0.0)
FACING = (0.0, 0.0, 0.0, 1.0)  # yaw 180 degrees: a tag facing the camera
HALF_TURN_AXES = (0.5, 0.5, 0.5, 0.5)
IDENTITY = (1.0, 0.0, 0.0, 0.0)


def case_empty() -> Case:
    return Case("empty", "No target and no multi-tag result: the result of a frame in which nothing was detected.",
                ("message PhotonPipelineResult", meta_line(1, 1000, 26000, 10000)))


def case_one_target() -> Case:
    best = transform(2.5, -0.35, 0.1, PITCH_30)
    alt = transform(2.5, -0.35, 0.1, PITCH_MINUS_20)
    det = [(598.5, 428.25), (657.0, 428.25), (656.5, 374.5), (599.75, 374.5)]
    rect = [(598.0, 429.0), (657.5, 429.0), (657.5, 374.0), (598.0, 374.0)]  # not the detected corners: catches swaps
    return Case("one_target", "One AprilTag target with both corner lists and no multi-tag result.",
                ("message PhotonPipelineResult", meta_line(42, 2475000, 2500000, 10000),
                 target_line(-3.5, 2.25, 0.75, 0.0, 7, -1, -1.0, 0.125, best, alt, rect, det)))


def case_three_targets_multitag() -> Case:
    # The shape of a real frame: three tags seen by a camera pitched up 20 degrees, then the multi-tag solve.
    scene = [
        (-1.083502, -0.08103, 0.305768, 26, 0.02, (1.831424, 0.034638, -0.002590),
         [(598.407013, 428.451512), (657.005381, 428.451512), (656.489022, 374.195146), (599.669963, 374.195146)]),
        (-12.028625, -0.08103, 0.305768, 25, 0.07, (1.831424, 0.390238, -0.002590),
         [(472.195144, 428.451512), (530.793512, 428.451512), (534.109510, 374.195146), (477.290450, 374.195146)]),
        (-2.573656, -16.039361, 0.015016, 3, 0.12, (8.681786, 0.390238, -2.495918),
         [(605.033543, 590.330317), (617.244020, 590.330317), (617.391548, 577.696688), (605.260233, 577.696688)]),
    ]
    lines = ["message PhotonPipelineResult", meta_line(7, 123431789, 123456789, 10000)]
    for yaw, pitch, area, fiducial_id, ambiguity, (x, y, z), det in scene:
        xs = [p[0] for p in det]
        ys = [p[1] for p in det]
        rect = [(min(xs), max(ys)), (max(xs), max(ys)), (max(xs), min(ys)), (min(xs), min(ys))]
        lines.append(target_line(yaw, pitch, area, 0.0, fiducial_id, -1, -1.0, ambiguity,
                                 transform(x, y, z, FACING), transform(x, y, z + 0.5, HALF_TURN_AXES), rect, det))
    field_to_camera = transform(2.3, 4.0, 0.5, PITCH_MINUS_20)
    lines.append(multi_line(0.25, 0.25, 0.0, field_to_camera, field_to_camera, [26, 25, 3]))
    return Case("three_targets_multitag", "Three targets with corners and a multi-tag result (alt = best, ambiguity 0).",
                tuple(lines))


def case_edge_values() -> Case:
    best = transform(1e-300, -1e300, 0.0, IDENTITY)
    alt = transform(-0.0, 5e-324, 123456789.123456789, HALF_TURN_AXES)
    return Case(
        "edge_values",
        "Extremes of every width: int64 / int32 / int16 limits, a negative zero, subnormal and huge doubles, "
        "list lengths other than four.",
        ("message PhotonPipelineResult",
         meta_line(9223372036854775807, -9223372036854775808, 4294967296, 0),
         target_line(-0.0, 5e-324, 1.7976931348623157e308, -1.5e-300, 2147483647, -2147483648, 3.4028234663852886e38,
                     -1.0, best, alt, [(-1.5, 0.0), (0.1, 0.2), (3.0, -4.0)],
                     [(-0.0, 1e-9), (1e9, -1e9), (0.5, 0.25), (7.0, 8.0), (9.0, 10.0)]),
         multi_line(1e-12, 123456789.125, 0.5, alt, best, [-32768, 32767, 0, -1])))


def case_max_targets() -> Case:
    lines = ["message PhotonPipelineResult", meta_line(1000, 5000000, 5025000, 10000)]
    rotations = (IDENTITY, HALF_TURN_AXES, (0.5, -0.5, 0.5, -0.5), (0.0, 0.0, 1.0, 0.0))
    for i in range(MAX_LIST):
        best = transform(i / 8, -i / 16, 0.5 + i / 32, rotations[i % 4])
        alt = transform(i / 8, -i / 16, 0.5 + i / 64, rotations[(i + 1) % 4])
        lines.append(target_line(i - 63, (i % 9) - 4, (i + 1) / 128, 0.0, i, -1, -1.0, (i % 11) / 16, best, alt, [], []))
    pose = transform(8.25, 4.125, 0.5, PITCH_MINUS_20)
    lines.append(multi_line(0.5, 0.5, 0.0, pose, pose, list(range(MAX_LIST))))
    return Case("max_targets", f"{MAX_LIST} targets (the longest list PhotonLib allows) and {MAX_LIST} multi-tag ids.",
                tuple(lines))


def case_max_corners() -> Case:
    det = [(i * 0.5, 1000 - i * 0.25) for i in range(MAX_LIST)]
    rect = [(i * 0.75, i * 1.5) for i in range(MAX_LIST)]
    pose = transform(0.0, 0.0, 1.0, IDENTITY)
    return Case("max_corners", f"One target with {MAX_LIST} detected and {MAX_LIST} min-area-rect corners.",
                ("message PhotonPipelineResult", meta_line(2, 3000, 4000, 10000),
                 target_line(1.0, -1.0, 2.0, 0.0, 5, -1, -1.0, 0.5, pose, pose, rect, det)))


def case_transform3d_pose() -> Case:
    return Case("transform3d_pose", "The payload of a struct:Transform3d topic (PhotonVision's targetPose).",
                ("message Transform3d", "value " + " ".join(map(num, transform(1.5, -2.25, 0.125, HALF_TURN_AXES)))))


def case_transform3d_identity() -> Case:
    return Case("transform3d_identity", "The targetPose of a frame without a target: Transform3d().",
                ("message Transform3d", "value " + " ".join(map(num, transform(0.0, 0.0, 0.0, IDENTITY)))))


def build_cases() -> list:
    return [case_empty(), case_one_target(), case_three_targets_multitag(), case_edge_values(), case_max_targets(),
            case_max_corners(), case_transform3d_pose(), case_transform3d_identity()]


# ---- the real library --------------------------------------------------------------------------------------------

class Schema(NamedTuple):
    name: str
    uuid: str
    text: str


class LibraryAnswer(NamedTuple):
    version: str
    max_list: int
    root_uuid: str
    schemas: tuple  # Schema, root first
    bytes_by_case: dict  # case name -> hex


def parse_java_output(text: str, case_names) -> LibraryAnswer:
    """The tab separated lines tools/gen_photon_fixtures.java prints."""
    version = max_list = None
    schemas = []
    packed = {}
    for line in text.splitlines():
        if not line.strip():
            continue
        fields = line.split("\t")
        kind = fields[0]
        if kind == "version" and len(fields) == 2:
            version = fields[1]
        elif kind == "maxlist" and len(fields) == 2:
            max_list = int(fields[1])
        elif kind == "schema" and len(fields) == 4:
            schemas.append(Schema(fields[1], fields[2], fields[3]))
        elif kind == "bytes" and len(fields) == 3:
            packed[fields[1]] = fields[2]
        else:
            raise GenerationError(f"unexpected output of the Java program: {line!r}")
    if version is None or max_list is None or not schemas:
        raise GenerationError("the Java program did not report the PhotonLib version and schemas")
    missing = [name for name in case_names if name not in packed]
    if missing:
        raise GenerationError("the Java program produced no bytes for: " + ", ".join(missing))
    return LibraryAnswer(version, max_list, schemas[0].uuid, tuple(schemas), packed)


def java_input(cases) -> str:
    out = []
    for case in cases:
        out.append(f"fixture {case.name}")
        out.extend(case.lines)
        out.append("end")
    return "\n".join(out) + "\n"


def run_java(java: Path, classpath: list, cases) -> LibraryAnswer:
    command = [str(java), "-cp", os.pathsep.join(str(path) for path in classpath), str(JAVA_PROGRAM)]
    try:
        done = subprocess.run(command, input=java_input(cases).encode("ascii"), capture_output=True, timeout=300)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise GenerationError(f"cannot run {java}: {error}") from error
    stderr = done.stderr.decode("utf-8", errors="replace")
    if done.returncode != 0:
        raise GenerationError(f"the Java program failed ({done.returncode}):\n{stderr.strip()}")
    return parse_java_output(done.stdout.decode("utf-8"), [case.name for case in cases])


# ---- the toolchain -------------------------------------------------------------------------------------------------

def read_vendordep(path: Path = VENDORDEP) -> dict:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        return {"version": document["version"], "year": str(document["frcYear"]),
                "urls": [url.rstrip("/") for url in document.get("mavenUrls", [])]}
    except (OSError, ValueError, KeyError) as error:
        raise GenerationError(f"{path}: cannot read the PhotonLib vendordep ({error})") from error


def sample_wpilib_version() -> str | None:
    try:
        match = re.search(r'GradleRIO"\s+version\s+"([0-9][0-9A-Za-z.\-]*)"', SAMPLE_GRADLE.read_text(encoding="utf-8"))
    except OSError:
        return None
    return match.group(1) if match else None


def wpilib_roots(year: str) -> list:
    roots = []
    if sys.platform == "win32":
        roots.append(Path(os.environ.get("PUBLIC", "C:/Users/Public")) / "wpilib" / year)
    roots.append(Path.home() / "wpilib" / year)
    return list(dict.fromkeys(roots))


def gradle_cache() -> Path:
    return Path(os.environ.get("GRADLE_USER_HOME", Path.home() / ".gradle")) / "caches" / "modules-2" / "files-2.1"


def find_java(explicit: Path | None, year: str) -> Path | None:
    exe = "java.exe" if sys.platform == "win32" else "java"
    candidates = [explicit] if explicit else []
    if not explicit:
        if os.environ.get("JAVA_HOME"):
            candidates.append(Path(os.environ["JAVA_HOME"]) / "bin" / exe)
        candidates += [root / "jdk" / "bin" / exe for root in wpilib_roots(year)]
        found = shutil.which("java")
        if found:
            candidates.append(Path(found))
    return next((path for path in candidates if path.is_file()), None)


def version_key(text: str) -> tuple:
    return tuple(int(part) if part.isdigit() else 0 for part in re.split(r"[.\-]", text.lstrip("v")))


def find_jar(directories, artifact: str, version: str | None) -> Path | None:
    """<artifact>-<version>.jar (the highest version when none is given) under the first directory that has one.

    Both layouts are understood: Maven (<artifact>/<version>/<artifact>-<version>.jar) and the Gradle cache
    (<artifact>/<version>/<hash>/<artifact>-<version>.jar)."""
    for directory in directories:
        if not directory.is_dir():
            continue
        best = None
        for jar in directory.rglob(f"{artifact}-*.jar"):
            jar_version = jar.name[len(artifact) + 1:-len(".jar")]
            if jar_version not in (jar.parent.name, jar.parent.parent.name):
                continue  # -sources, -javadoc and platform classifiers do not name their directory
            if version is not None and jar_version != version:
                continue
            if best is None or version_key(jar_version) > version_key(best[0]):
                best = (jar_version, jar)
        if best is not None:
            return best[1]
    return None


def download(url: str, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        with urllib.request.urlopen(url, timeout=120) as response:
            data = response.read()
    except (urllib.error.URLError, OSError) as error:
        raise GenerationError(f"cannot download {url}: {error}") from error
    target.write_bytes(data)


def photon_jar(artifact: str, version: str, urls: list, explicit: Path | None, allow_download: bool) -> Path:
    wanted = f"{artifact}-{version}.jar"
    if explicit is not None:
        sibling = explicit if explicit.name == wanted else explicit.parent / wanted
        if sibling.is_file():
            return sibling
    found = find_jar([gradle_cache() / "org.photonvision" / artifact / version, DOWNLOAD_DIR], artifact, version)
    if found is not None:
        return found
    if not allow_download:
        raise ToolsMissing(f"{wanted} is not installed (build samples/vision_robot/java once, or pass --photon-jar "
                           "<path>, or --download)")
    problems = []
    for url in urls:
        target = DOWNLOAD_DIR / wanted
        try:
            download(f"{url}/org/photonvision/{artifact}/{version}/{wanted}", target)
            return target
        except GenerationError as error:
            problems.append(str(error))
    raise GenerationError("; ".join(problems) or "the vendordep lists no Maven URL")


def library_jars(year: str, wpilib_maven: Path | None, wpilib_version: str | None) -> list:
    """The WPILib and third-party jars below PhotonLib: the WPILib Maven directory first, then the Gradle cache."""
    maven_roots = [wpilib_maven] if wpilib_maven else [root / "maven" for root in wpilib_roots(year)]
    jars = []
    for group, artifact, required, wpilib_own in LIBRARIES:
        directories = [root / group.replace(".", "/") / artifact for root in maven_roots]
        directories.append(gradle_cache() / group / artifact)
        found = None
        if wpilib_own and wpilib_version:
            found = find_jar(directories, artifact, wpilib_version)
        found = found or find_jar(directories, artifact, None)
        if found is None and required:
            raise ToolsMissing(f"{artifact} is not installed (searched {', '.join(str(d) for d in directories)}; "
                               "use --wpilib-maven <dir>)")
        if found is not None:
            jars.append(found)
    return jars


# ---- the files -------------------------------------------------------------------------------------------------

def render_fixture(case: Case, answer: LibraryAnswer) -> str:
    header = [f"# Golden bytes of {case.lines[0].split()[1]} from PhotonLib {answer.version} (photontargeting-java): "
              "the output of the real serde.",
              "# Generated by tools/gen_photon_fixtures.py. Do not edit; regenerate (see the header of that tool).",
              f"# {case.description}"]
    return "\n".join([*header, *case.lines, f"bytes {answer.bytes_by_case[case.name]}"]) + "\n"


def render_schemas(answer: LibraryAnswer) -> str:
    lines = [f"# PhotonVision message definitions of PhotonLib {answer.version} (photontargeting-java), as the real "
             "library reports them:",
             "# name<TAB>uuid<TAB>schema text; the uuid is the MD5 of the text. The first message is the result that "
             "travels on rawBytes.",
             "# Generated by tools/gen_photon_fixtures.py. Do not edit; regenerate (see the header of that tool).",
             f"version\t{answer.version}", f"maxlist\t{answer.max_list}"]
    lines += [f"{schema.name}\t{schema.uuid}\t{schema.text}" for schema in answer.schemas]
    return "\n".join(lines) + "\n"


def render_files(cases, answer: LibraryAnswer) -> dict:
    """relative file name -> text, below <out>/<root uuid>/."""
    files = {"schemas.txt": render_schemas(answer)}
    for case in cases:
        files[f"{case.name}.fixture"] = render_fixture(case, answer)
    for name, text in files.items():
        text.encode("ascii")  # the fixtures are plain ASCII
    return files


def display(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO).as_posix()
    except ValueError:
        return str(path)


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                     epilog="Exit status: 0 done / up to date, 1 --check found a difference, "
                                            "2 unusable input or failure, 77 --check skipped (JDK or jars missing).")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help=f"fixture root (default: {display(DEFAULT_OUT)})")
    parser.add_argument("--check", action="store_true", help="regenerate in memory and compare with --out instead of writing")
    parser.add_argument("--photonlib-version", help="release to run, e.g. v2026.3.4 (default: the sample's vendordep)")
    parser.add_argument("--java", type=Path, help="the java executable of a JDK 17+")
    parser.add_argument("--photon-jar", type=Path, help="photontargeting-java-<version>.jar (photonlib-java next to it)")
    parser.add_argument("--wpilib-maven", type=Path, help="the WPILib Maven directory with the WPILib Java jars")
    parser.add_argument("--wpilib-version", help="WPILib version of those jars (default: the sample's GradleRIO)")
    parser.add_argument("--download", action="store_true",
                        help="download missing PhotonLib jars from the vendordep's Maven URLs into build/photon-fixture-jars")
    return parser.parse_args(argv)


def generate(args) -> tuple:
    """(root UUID, files) from the real library; raises ToolsMissing / GenerationError."""
    vendordep = read_vendordep()
    version = args.photonlib_version or vendordep["version"]
    year = vendordep["year"]
    java = find_java(args.java, year)
    if java is None:
        raise ToolsMissing("no JDK found (set JAVA_HOME, install WPILib, or pass --java)")
    classpath = [photon_jar(artifact, version, vendordep["urls"], args.photon_jar, args.download)
                 for artifact in PHOTON_ARTIFACTS]
    classpath += library_jars(year, args.wpilib_maven, args.wpilib_version or sample_wpilib_version())
    cases = build_cases()
    answer = run_java(java, classpath, cases)
    if answer.max_list != MAX_LIST:
        raise GenerationError(f"PhotonLib's list limit is {answer.max_list}, the cases assume {MAX_LIST}")
    return answer.root_uuid, render_files(cases, answer)


def differences(directory: Path, files: dict) -> list:
    problems = []
    for name, text in files.items():
        path = directory / name
        if not path.is_file():
            problems.append(f"{display(path)} does not exist")
            continue
        committed = path.read_bytes().decode("utf-8").replace("\r\n", "\n")  # a CRLF checkout is not a difference
        if committed != text:
            diff = list(difflib.unified_diff(committed.splitlines(), text.splitlines(), "committed", "regenerated",
                                             lineterm="", n=1))
            problems.append(f"{display(path)} is out of date:\n" + "\n".join(line[:160] for line in diff[:DIFF_LINES]))
    if directory.is_dir():
        for path in sorted(directory.glob("*.fixture")):
            if path.name not in files:
                problems.append(f"{display(path)} is not produced any more (delete it)")
    return problems


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        uuid, files = generate(args)
    except ToolsMissing as error:
        if args.check:
            print(f"SKIPPED: {error}, so the PhotonVision fixtures cannot be checked")
            return SKIPPED
        print(f"error: {error}", file=sys.stderr)
        return 2
    except GenerationError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    directory = args.out / uuid
    regenerate = "python tools/gen_photon_fixtures.py"
    if args.check:
        problems = differences(directory, files)
        if problems:
            print(f"error: the PhotonVision fixtures in {display(directory)} are out of date:\n" +
                  "\n".join(problems) + f"\nRegenerate with: {regenerate}", file=sys.stderr)
            return 1
        print(f"{display(directory)} is up to date ({len(files)} files, message UUID {uuid})")
        return 0

    try:
        directory.mkdir(parents=True, exist_ok=True)
        for stale in directory.glob("*.fixture"):
            if stale.name not in files:
                stale.unlink()
        for name, text in files.items():
            (directory / name).write_bytes(text.encode("ascii"))  # bytes: LF endings on every platform
    except OSError as error:
        print(f"error: cannot write {directory} ({error})", file=sys.stderr)
        return 2
    print(f"wrote {display(directory)} ({len(files)} files, message UUID {uuid})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
