#!/usr/bin/env python3
"""Season facts for workflows and packaging scripts, read from seasons/<year>/season.json.

CI, the release workflow and the packaging scripts call this instead of repeating season years:
  seasons.py years                          the season years, space separated
  seasons.py extensions --layout <layout>   the install-relative path of every season's HALSim extension, one per line
      layouts: windows | linux | macos (relative to the install prefix) | macos-bundle (relative to wpiemu_app.app)
Run:  seasons.py [--root DIR] <command>
"""
from __future__ import annotations

import argparse
import sys

import common

LAYOUTS = {
    "windows": "extensions/{year}/halsim_wpiemu.dll",
    "linux": "lib/wpiemu/{year}/libhalsim_wpiemu.so",
    "macos": "wpiemu_app.app/Contents/PlugIns/wpiemu/{year}/libhalsim_wpiemu.dylib",
    "macos-bundle": "Contents/PlugIns/wpiemu/{year}/libhalsim_wpiemu.dylib",
}


def years(root) -> list[str]:
    return list(common.load_seasons(root))


def extensions(root, layout: str) -> list[str]:
    return [LAYOUTS[layout].format(year=year) for year in years(root)]


def main() -> int:
    parser = common.root_parser(__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("years")
    ext = sub.add_parser("extensions")
    ext.add_argument("--layout", choices=sorted(LAYOUTS), required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    if args.command == "years":
        print(" ".join(years(root)))
    else:
        print("\n".join(extensions(root, args.layout)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
