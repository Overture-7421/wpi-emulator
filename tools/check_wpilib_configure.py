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
    parser.add_argument("--directory", type=Path, default=Path("build/wpilib-root-fixture"))
    args = parser.parse_args()
    repo = common.REPO_ROOT
    work = args.directory.resolve()
    extracted = work / "extracted"
    headers = {"hal/HAL.h": "", "hal/Ports.h": "", "hal/simulation/SimDeviceData.h": "",
               "wpi/string.h": ""}
    libraries = {}
    for platform, names in (
            ("windows/x86-64", ("wpiHal.dll", "wpiHal.lib", "wpiutil.dll", "wpiutil.lib")),
            ("osx/universal", ("libwpiHal.dylib", "libwpiutil.dylib")),
            ("linux/x86-64", ("libwpiHal.so", "libwpiutil.so")),
            ("linux/arm64", ("libwpiHal.so", "libwpiutil.so"))):
        libraries.update({f"{platform}/shared/{name}": "" for name in names})
    for name, content in {**{f"include/{key}": value for key, value in headers.items()},
                          **libraries}.items():
        common.write_text(extracted / name, content)
    maven = work / "maven"
    for module in ("hal", "wpiutil"):
        prefix = maven / f"edu/wpi/first/{module}/{module}-cpp/2026.2.1/{module}-cpp-2026.2.1"
        prefix.parent.mkdir(parents=True, exist_ok=True)
        for classifier, entries in (
                ("headers", {key: value for key, value in headers.items()
                             if key.startswith("hal/" if module == "hal" else "wpi/")}),
                *[(classifier, {key: value for key, value in libraries.items()
                               if key.startswith(platform + "/") and
                               ("wpiHal" in key if module == "hal" else "wpiutil" in key)})
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
                           f"-DWPIEMU_BUILD_TESTS={tests}"]
                if args.generator:
                    command += ["-G", args.generator]
                if args.architecture:
                    command += ["-A", args.architecture]
                subprocess.run(command, env=env, check=True)
                print(f"WPILib configure: {case}: ok", flush=True)


if __name__ == "__main__":
    main()
