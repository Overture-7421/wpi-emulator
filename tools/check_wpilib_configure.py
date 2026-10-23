#!/usr/bin/env python3
"""Offline configure-only regression matrix for WPILib root discovery and bridge selection.

Uses synthetic headers/libraries, never builds or downloads. Run in its own build tree:
  python tools\\check_wpilib_configure.py --generator "Visual Studio 17 2022" --architecture x64
"""
import argparse
import os
import subprocess
import zipfile
from pathlib import Path

import common


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generator")
    parser.add_argument("--architecture")
    parser.add_argument("--season", help="Check one profile instead of every configured season")
    parser.add_argument("--directory", type=Path, default=Path("build/wpilib-root-fixture"))
    args = parser.parse_args()
    repo = common.REPO_ROOT
    wpilib_pin = common.load_versions(repo)["wpilib"]
    if args.season and args.season not in wpilib_pin["seasons"]:
        parser.error(f"unknown season {args.season}")
    for season in wpilib_pin["seasons"]:
        if args.season and args.season != season:
            continue
        check_season(args, repo, season, wpilib_pin["seasons"][season])


def check_season(args, repo: Path, season: str, profile: dict) -> None:
    wpilib = profile["version"]
    header_roots = profile["headers"]
    work = args.directory.resolve() / season
    extracted = work / "extracted"
    hal_header_dir = Path(header_roots["hal"]).parent.as_posix()
    headers = {header_roots["hal"]: "", f"{hal_header_dir}/Ports.h": "",
               f"{hal_header_dir}/simulation/SimDeviceData.h": "", header_roots["wpiutil"]: "",
               header_roots["ntcore"]: ""}
    libraries = {}
    modules = {"hal": "wpiHal", "wpiutil": "wpiutil", "ntcore": "ntcore", "wpinet": "wpinet"}
    if int(season) >= 2027:
        modules["datalog"] = "datalog"
    for platform, extensions, prefix in (
            ("windows/x86-64", ("dll", "lib"), ""),
            ("osx/universal", ("dylib",), "lib"),
            ("linux/x86-64", ("so",), "lib"),
            ("linux/arm64", ("so",), "lib")):
        for name in modules.values():
            libraries.update({f"{platform}/shared/{prefix}{name}.{ext}": "" for ext in extensions})
    for name, content in {**{f"include/{key}": value for key, value in headers.items()},
                          **libraries}.items():
        common.write_text(extracted / name, content)
    maven = work / "maven"
    for module, library in modules.items():
        group = profile["groups"][module].replace(".", "/")
        artifact = module + "-cpp"
        prefix = maven / group / artifact / wpilib / f"{artifact}-{wpilib}"
        prefix.parent.mkdir(parents=True, exist_ok=True)
        for classifier, entries in (
                ("headers", {key: value for key, value in headers.items()
                             if key == header_roots.get(module) or
                             (module == "hal" and key.startswith(hal_header_dir + "/"))}),
                *[(classifier, {key: value for key, value in libraries.items()
                               if key.startswith(platform + "/") and
                               library in key})
                  for classifier, platform in (("windowsx86-64", "windows/x86-64"),
                                               ("osxuniversal", "osx/universal"),
                                               ("linuxx86-64", "linux/x86-64"),
                                               ("linuxarm64", "linux/arm64"))]):
            with zipfile.ZipFile(f"{prefix}-{classifier}.zip", "w") as archive:
                for name, content in entries.items():
                    archive.writestr(name, content)
    for layout, root in (("extracted", extracted), ("maven", maven)):
        for route in ("cache", "environment"):
            for tests in ("OFF", "ON"):
                case = f"{layout}-{route}-tests-{tests}"
                env = os.environ.copy()
                env.pop("WPIEMU_WPILIB_ROOT", None)
                cache_root = root.as_posix() if route == "cache" else ""
                if route == "environment":
                    env["WPIEMU_WPILIB_ROOT"] = root.as_posix()
                command = ["cmake", "-S", str(repo / "cmake/tests/wpilib"),
                           "-B", str(work / case), f"-DWPIEMU_FIXTURE_REPO={repo.as_posix()}",
                           f"-DWPIEMU_FIXTURE_LAYOUT={layout}", f"-DWPIEMU_FIXTURE_ROOT={root.as_posix()}",
                           f"-DWPIEMU_FIXTURE_CACHE_ROOT={cache_root}",
                           f"-DWPIEMU_WPILIB_ROOT={cache_root}", "-DWPIEMU_WPILIB_DOWNLOAD=OFF",
                           f"-DWPIEMU_WPILIB_SEASON={season}",
                           f"-DWPIEMU_BUILD_TESTS={tests}"]
                if args.generator:
                    command += ["-G", args.generator]
                if args.architecture:
                    command += ["-A", args.architecture]
                subprocess.run(command, env=env, check=True)
                print(f"WPILib configure: {season}/{case}: ok", flush=True)


if __name__ == "__main__":
    main()
