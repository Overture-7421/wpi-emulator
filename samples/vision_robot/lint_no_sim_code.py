#!/usr/bin/env python3
"""Fails when the vision sample robots contain simulation-specific code.

The sample robots must stay plain WPILib projects: every camera frame comes from the emulator,
never from robot-side simulation. This scans <root>/java/src and <root>/cpp/src for the vendor
and WPILib simulation APIs that would bypass the emulator and exits non-zero (printing
file:line) when it finds one.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

FORBIDDEN = (
    "PhotonCameraSim",
    "VisionSystemSim",
    "SimCameraProperties",
    "simulationPeriodic",
    "simulationInit",
    "SimDeviceSim",
    "SimPhotonVision",
    "SimulatedArena",
    "maplesim",
)
SOURCE_DIRS = ("java/src", "cpp/src")

# Case-insensitive so the C++ spellings (SimulationPeriodic, SimulationInit) are caught too.
PATTERN = re.compile("|".join(re.escape(word) for word in FORBIDDEN), re.IGNORECASE)


def scan(root: Path) -> tuple[int, list[str]]:
    """Returns the number of files scanned and one "file:line: message" entry per finding."""
    scanned = 0
    findings: list[str] = []
    for source_dir in SOURCE_DIRS:
        for path in sorted((root / source_dir).rglob("*")):
            if not path.is_file():
                continue
            scanned += 1
            text = path.read_text(encoding="utf-8", errors="replace")
            for number, line in enumerate(text.splitlines(), start=1):
                for match in PATTERN.finditer(line):
                    location = path.relative_to(root).as_posix()
                    findings.append(f"{location}:{number}: forbidden simulation API '{match.group(0)}'")
    return scanned, findings


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parent,
        help="directory containing java/ and cpp/ (default: the directory of this script)",
    )
    args = parser.parse_args(argv)

    missing = [d for d in SOURCE_DIRS if not (args.root / d).is_dir()]
    if missing:
        print(f"error: missing source directories under {args.root}: {', '.join(missing)}", file=sys.stderr)
        return 2

    scanned, findings = scan(args.root)
    for finding in findings:
        print(finding)
    if findings:
        print(f"FAIL: {len(findings)} forbidden reference(s) in {scanned} files", file=sys.stderr)
        return 1
    print(f"OK: no simulation-specific code in {scanned} files")
    return 0


if __name__ == "__main__":
    sys.exit(main())
