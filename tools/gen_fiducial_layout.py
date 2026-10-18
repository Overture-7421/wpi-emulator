#!/usr/bin/env python3
"""Generate plugins/game_2026/src/fiducial_layouts.cpp from WPILib's AprilTag layout resources.

The REBUILT (2026) AprilTag layouts ship inside the WPILib ``apriltag-java`` jar as JSON resources (a jar is a
zip). This tool reads the two resources, validates them and writes ONE C++ source: constexpr tables of
``sdk::FiducialTag`` plus ``buildFiducialLayouts()``. Every number is the JSON's own decimal text, so the tables
are an exact transcription (the compiler rounds each literal to the nearest float).

    python tools/gen_fiducial_layout.py                 # rewrite the checked-in source
    python tools/gen_fiducial_layout.py --check         # fail when the checked-in source is out of date
    python tools/gen_fiducial_layout.py --jar <jar>     # WPILib is not under its default location

Exit status: 0 done / up to date, 1 --check found a difference, 2 unusable input or options, 77 --check was
skipped because the default jar is not installed (the CTest "skip" convention). Standard library only.
"""
from __future__ import annotations

import argparse
import difflib
import hashlib
import json
import math
import os
import re
import struct
import sys
import zipfile
from decimal import Decimal
from pathlib import Path
from typing import NamedTuple

REPO = Path(__file__).resolve().parents[1]
DEFAULT_OUT = REPO / "plugins" / "game_2026" / "src" / "fiducial_layouts.cpp"

WPILIB_YEAR = "2026"
WPILIB_VERSION = "2026.2.1"
JAR_NAME = re.compile(r"apriltag-java-(?P<version>[0-9][0-9A-Za-z.\-]*)\.jar")
RESOURCE_DIR = "edu/wpi/first/apriltag/"
TAG_SIZE_M = "0.1651"  # FIRST AprilTag edge, 6.5 in; the JSON carries no tag size
SKIPPED = 77
DIFF_LINES = 40


class LayoutSpec(NamedTuple):
    id: str  # FiducialLayout::id, "<pluginId>.<name>"
    display_name: str
    resource: str  # path inside the jar
    symbol: str  # C++ table name suffix


# The FIRST entry is the plugin default (IGameModule::contributeFiducialLayouts): welded, which is also the
# layout docs/game_2026.md takes the physical field model from.
LAYOUTS = (
    LayoutSpec("game_2026.welded", "REBUILT 2026 (welded)", RESOURCE_DIR + "2026-rebuilt-welded.json", "Welded"),
    LayoutSpec("game_2026.andymark", "REBUILT 2026 (AndyMark)", RESOURCE_DIR + "2026-rebuilt-andymark.json",
               "AndyMark"),
)


class Tag(NamedTuple):
    id: int
    position: tuple  # x, y, z in metres (Decimal)
    rotation: tuple  # quaternion x, y, z, w (Decimal)


class Layout(NamedTuple):
    spec: LayoutSpec
    sha256: str  # of the resource bytes as stored in the jar
    length_m: Decimal
    width_m: Decimal
    tags: tuple  # sorted by id


class GenerationError(Exception):
    """The jar or one of its resources cannot be turned into a layout table."""


def default_jar_candidates() -> list[Path]:
    """Where the WPILib installer puts its Maven cache: %PUBLIC% on Windows, the home directory elsewhere."""
    roots = []
    if sys.platform == "win32":
        roots.append(Path(os.environ.get("PUBLIC", "C:/Users/Public")) / "wpilib" / WPILIB_YEAR / "maven")
    roots.append(Path.home() / "wpilib" / WPILIB_YEAR / "maven")
    relative = f"edu/wpi/first/apriltag/apriltag-java/{WPILIB_VERSION}/apriltag-java-{WPILIB_VERSION}.jar"
    return list(dict.fromkeys(root / relative for root in roots))


def _reject_constant(name: str):
    raise ValueError(f"{name} is not a finite number")


def _member(obj, key: str, where: str):
    if not isinstance(obj, dict) or key not in obj:
        raise GenerationError(f"{where}: missing '{key}'")
    return obj[key]


def _number(value, where: str) -> Decimal:
    """A finite JSON number that survives the narrowing to float, kept as its exact decimal text."""
    if isinstance(value, bool) or not isinstance(value, (int, Decimal)):
        raise GenerationError(f"{where}: expected a number")
    number = Decimal(value)
    if not number.is_finite():
        raise GenerationError(f"{where}: expected a finite number")
    try:
        narrowed = struct.unpack("<f", struct.pack("<f", float(number)))[0]
    except OverflowError:
        narrowed = math.inf
    if not math.isfinite(narrowed):
        raise GenerationError(f"{where}: {number} does not fit a float")
    if narrowed == 0.0 and number != 0:
        raise GenerationError(f"{where}: {number} underflows a float")
    return number


def parse_layout(spec: LayoutSpec, raw: bytes) -> Layout:
    where = spec.resource
    try:
        document = json.loads(raw.decode("utf-8"), parse_float=Decimal, parse_constant=_reject_constant)
    except (UnicodeDecodeError, ValueError) as error:
        raise GenerationError(f"{where}: not valid JSON ({error})") from error
    field = _member(document, "field", where)
    length = _number(_member(field, "length", f"{where} field"), f"{where} field.length")
    width = _number(_member(field, "width", f"{where} field"), f"{where} field.width")
    if length <= 0 or width <= 0:
        raise GenerationError(f"{where}: the field size must be positive")
    entries = _member(document, "tags", where)
    if not isinstance(entries, list) or not entries:
        raise GenerationError(f"{where}: 'tags' must be a non-empty array")

    tags = []
    for index, entry in enumerate(entries):
        here = f"{where} tags[{index}]"
        ident = _member(entry, "ID", here)
        if isinstance(ident, bool) or not isinstance(ident, int) or ident < 1:
            raise GenerationError(f"{here}: ID must be a positive integer")
        pose = _member(entry, "pose", here)
        translation = _member(pose, "translation", f"{here} pose")
        quaternion = _member(_member(pose, "rotation", f"{here} pose"), "quaternion", f"{here} pose.rotation")
        position = tuple(_number(_member(translation, axis, f"{here} translation"), f"{here} translation.{axis}")
                         for axis in "xyz")
        rotation = tuple(_number(_member(quaternion, axis, f"{here} quaternion"), f"{here} quaternion.{axis}")
                         for axis in "XYZW")
        norm = math.sqrt(sum(float(c) ** 2 for c in rotation))
        if abs(norm - 1.0) > 1e-6:
            raise GenerationError(f"{here}: the quaternion is not unit length (norm {norm})")
        tags.append(Tag(ident, position, rotation))
    ids = [tag.id for tag in tags]
    if len(set(ids)) != len(ids):
        raise GenerationError(f"{where}: duplicate tag IDs")
    tags.sort(key=lambda tag: tag.id)
    return Layout(spec, hashlib.sha256(raw).hexdigest(), length, width, tuple(tags))


def read_layouts(jar: Path) -> tuple[str, list[Layout]]:
    """The jar's version (from its file name; the jar has no other metadata) and the parsed layouts."""
    match = JAR_NAME.fullmatch(jar.name)
    if match is None:
        raise GenerationError(f"{jar}: expected a file named apriltag-java-<version>.jar "
                              "(the version is recorded in the provenance)")
    layouts = []
    try:
        with zipfile.ZipFile(jar) as archive:
            for spec in LAYOUTS:
                try:
                    raw = archive.read(spec.resource)
                except KeyError:
                    raise GenerationError(f"{jar.name}: resource {spec.resource} not found") from None
                layouts.append(parse_layout(spec, raw))
    except (OSError, zipfile.BadZipFile) as error:
        raise GenerationError(f"{jar}: cannot read the jar ({error})") from error
    return match.group("version"), layouts


def cpp_float(number: Decimal) -> str:
    """The JSON decimal text as a float literal."""
    text = str(number).lower()
    if "." not in text and "e" not in text:
        text += ".0"
    return text + "f"


def render_tag(tag: Tag) -> str:
    x, y, z = (cpp_float(v) for v in tag.position)
    qx, qy, qz, qw = (cpp_float(v) for v in tag.rotation)
    return ("    {" + str(tag.id) + ", {{" + f"{x}, {y}, {z}" + "},\n" +
            "         {" + f"{qx}, {qy}, {qz}, {qw}" + "}}, 0.0},")


HEADER = """\
// GENERATED FILE - DO NOT EDIT. Regenerate with tools/gen_fiducial_layout.py, never patch by hand.
// clang-format off
//
// REBUILT (2026) AprilTag layouts for the vision simulation: where every fiducial is on the field
// ([AT] in docs/game_2026.md).
//
// Source: edu.wpi.first.apriltag:apriltag-java:@VERSION@ (@JAR@),
// WPILib's AprilTagFieldLayout resources, read as shipped:
@RESOURCES@//
// Frame: WPILib's blue-origin field frame IS the SDK world frame (+X from the blue wall to the red
// wall, +Y left, origin at the blue/low-Y corner of the carpet), so translations (m) and
// quaternions are copied unchanged (JSON X, Y, Z, W -> x, y, z, w). Tag frame: origin at the tag
// centre, +X = the outward normal, +Z up. Numbers are the JSON decimal text verbatim; the compiler
// rounds each one to the nearest float. Tag size: the JSON has none; FIRST AprilTags are 6.5 in =
// 0.1651 m.
//
// Regenerate (repository root; add --jar <path to the jar> if WPILib is not in its default
// location, %PUBLIC%/wpilib/2026/maven on Windows or ~/wpilib/2026/maven elsewhere):
//   python tools/gen_fiducial_layout.py
// Check that this file is current (skipped when the jar is not installed):
//   python tools/gen_fiducial_layout.py --check

#include "wpiemu/plugins/game_2026/fiducial_layouts.h"

#include <span>
#include <vector>

#include "wpiemu/sdk/vision/fiducial_layout.h"

namespace wpiemu::plugins::game_2026 {

namespace {

constexpr double kTagSizeM = @TAG_SIZE@;  // 6.5 in

// Rows: {id, {{x, y, z}, {qx, qy, qz, qw}}, sizeM}; a sizeM of 0 means FiducialLayout::tagSizeM.
"""

BUILDER = """\
sdk::FiducialLayout makeLayout(const char* id, const char* displayName, float fieldLengthM,
                               float fieldWidthM, std::span<const sdk::FiducialTag> tags) {
    sdk::FiducialLayout layout;
    layout.id = id;
    layout.displayName = displayName;
    layout.tagSizeM = kTagSizeM;
    layout.fieldLengthM = fieldLengthM;
    layout.fieldWidthM = fieldWidthM;
    layout.tags.assign(tags.begin(), tags.end());
    return layout;
}

}  // namespace

std::vector<sdk::FiducialLayout> buildFiducialLayouts() {
    std::vector<sdk::FiducialLayout> layouts;
    layouts.reserve(@COUNT@);
    // The first layout is the plugin default (VisionConfig::layoutId empty).
"""


def render(jar_name: str, version: str, layouts: list[Layout]) -> str:
    resources = "".join(
        f"//   {layout.spec.resource}\n"
        f"//     {len(layout.tags)} tags, field {layout.length_m} x {layout.width_m} m\n"
        f"//     sha256 {layout.sha256}\n" for layout in layouts)
    out = [HEADER.replace("@VERSION@", version).replace("@JAR@", jar_name).replace("@RESOURCES@", resources)
           .replace("@TAG_SIZE@", TAG_SIZE_M)]
    for layout in layouts:
        out.append(f"// {layout.spec.resource}\n")
        out.append(f"constexpr sdk::FiducialTag k{layout.spec.symbol}Tags[] = {{\n")
        out.append("\n".join(render_tag(tag) for tag in layout.tags) + "\n};\n\n")
    out.append(BUILDER.replace("@COUNT@", str(len(layouts))))
    for layout in layouts:
        out.append(f'    layouts.push_back(makeLayout("{layout.spec.id}", "{layout.spec.display_name}",\n')
        out.append(f"                                 {cpp_float(layout.length_m)}, {cpp_float(layout.width_m)}, "
                   f"k{layout.spec.symbol}Tags));\n")
    out.append("    return layouts;\n}\n\n}  // namespace wpiemu::plugins::game_2026\n")
    text = "".join(out)
    text.encode("ascii")  # the generated source is plain ASCII
    return text


def display(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO).as_posix()
    except ValueError:
        return str(path)


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                     epilog="Exit status: 0 done / up to date, 1 --check found a difference, "
                                            "2 unusable input, 77 --check skipped (default jar not installed).")
    parser.add_argument("--jar", type=Path,
                        help=f"apriltag-java-<version>.jar (default: the WPILib {WPILIB_YEAR} Maven cache)")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT,
                        help=f"generated source (default: {display(DEFAULT_OUT)})")
    parser.add_argument("--check", action="store_true",
                        help="regenerate in memory and compare with --out instead of writing it")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    jar = args.jar
    if jar is None:
        candidates = default_jar_candidates()
        jar = next((path for path in candidates if path.is_file()), None)
        if jar is None:
            searched = "".join(f"\n  searched: {path}" for path in candidates)
            problem = f"apriltag-java-{WPILIB_VERSION}.jar is not installed"
            hint = "\n  pass --jar <path> to use another copy"
            if args.check:
                print(f"SKIPPED: {problem}, so {display(args.out)} cannot be checked{searched}{hint}")
                return SKIPPED
            print(f"error: {problem}{searched}{hint}", file=sys.stderr)
            return 2
    elif not jar.is_file():
        print(f"error: --jar {jar}: no such file", file=sys.stderr)
        return 2

    try:
        version, layouts = read_layouts(jar)
        text = render(jar.name, version, layouts)
    except GenerationError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    summary = ", ".join(f"{layout.spec.id}: {len(layout.tags)} tags" for layout in layouts)

    if not args.check:
        try:
            args.out.write_bytes(text.encode("ascii"))  # bytes: LF endings on every platform
        except OSError as error:
            print(f"error: cannot write {args.out} ({error})", file=sys.stderr)
            return 2
        print(f"wrote {display(args.out)} ({summary}; {jar.name})")
        return 0

    regenerate = "python tools/gen_fiducial_layout.py"
    if not args.out.is_file():
        print(f"error: {display(args.out)} does not exist; run: {regenerate}", file=sys.stderr)
        return 1
    # A Windows checkout with core.autocrlf rewrites the line endings; they are not a difference.
    current = args.out.read_bytes().decode("utf-8", errors="replace").replace("\r\n", "\n")
    if current == text:
        print(f"{display(args.out)} is up to date ({summary}; {jar.name})")
        return 0
    diff = list(difflib.unified_diff(current.splitlines(), text.splitlines(), "checked-in", "regenerated",
                                     lineterm="", n=1))
    print(f"error: {display(args.out)} is out of date with {jar.name}; run: {regenerate}", file=sys.stderr)
    for line in diff[:DIFF_LINES]:
        print(line, file=sys.stderr)
    if len(diff) > DIFF_LINES:
        print(f"... ({len(diff) - DIFF_LINES} more diff lines)", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
