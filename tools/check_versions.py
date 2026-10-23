#!/usr/bin/env python3
"""Check that every version pin the build cannot read from versions.json repeats its value.

versions.json is the single home of the repository's version pins (docs/guides/wpilib-upgrade.md).
CMake and the Python tools read it directly; the workflows, the Docker image, the AppImage script
and the documentation cannot, so each copy listed in PINS is matched against it and any disagreement
is reported as file:line.
Run:  check_versions.py [--root DIR]
"""
from __future__ import annotations

import re
import sys
from pathlib import Path
from typing import NamedTuple

import common


class Pin(NamedTuple):
    key: str                 # dotted path into versions.json
    paths: tuple[str, ...]   # files or globs relative to the root; each must hold the pin
    pattern: str             # every match must hold the value; its groups, joined by ".", are the value found
    what: str


WORKFLOWS = (".github/workflows/ci.yml", ".github/workflows/release.yml")
QT_KIT_PATH = r"Qt[\\/](\d+\.\d+\.\d+)[\\/]"
UBUNTU = r"(?i)\bubuntu[-: ](\d+\.\d+)"

PINS = [
    Pin("wpilib", WORKFLOWS, r'^\s*WPILIB_VERSION:\s*"([^"]+)"', "WPILIB_VERSION (WPILib download cache key)"),
    Pin("wpilib", ("samples/*/*/build.gradle",), r'edu\.wpi\.first\.GradleRIO"\s+version\s+"([^"]+)"',
        "GradleRIO version of the sample robot"),
    Pin("qt.release", (".github/workflows/release.yml",), r'^\s*QT_VERSION:\s*"([^"]+)"', "QT_VERSION"),
    Pin("qt.release", ("tools/docker/linux/Dockerfile",), r"^ARG QT_VERSION=(\S+)", "QT_VERSION build argument"),
    Pin("qt.release", ("tools/docker/linux/Dockerfile",), r"\bQt (\d+\.\d+\.\d+)", "Qt version in the description"),
    Pin("qt.release", ("docs/BUILDING.md", "docs/releasing.md", "packaging/linux/build-appimage.sh"), QT_KIT_PATH,
        "Qt kit version in an example path"),
    Pin("qt.ci", (".github/workflows/ci.yml",), r'^\s+version:\s*"([^"]+)"', "Qt version pattern of install-qt-action"),
    Pin("python", WORKFLOWS, r'^\s+python-version:\s*"([^"]+)"', "python-version"),
    Pin("cmake", ("CMakeLists.txt", "cmake/tests/wpilib/CMakeLists.txt", "app/editor/standalone/CMakeLists.txt"),
        r"^cmake_minimum_required\(VERSION (\d+\.\d+)\)", "cmake_minimum_required"),
    Pin("cmake", ("CMakePresets.json",), r'"cmakeMinimumRequired":\s*\{\s*"major":\s*(\d+),\s*"minor":\s*(\d+)',
        "cmakeMinimumRequired"),
    Pin("cmake", ("docs/BUILDING.md",), r"CMake (\d+\.\d+)\+", "minimum CMake in the prerequisites"),
    Pin("cmake", ("tools/docker/linux/Dockerfile",), r"presets need (\d+\.\d+)\+", "minimum CMake for the presets"),
    Pin("cmake", ("packaging/linux/build-appimage.sh",), r"cmake >= (\d+\.\d+)", "minimum CMake in the requirements"),
    Pin("ubuntu", (".github/workflows/release.yml", "tools/docker/linux/Dockerfile", "tools/docker/linux/run.ps1",
                   "packaging/linux/build-appimage.sh", "docs/releasing.md"), UBUNTU, "Ubuntu release"),
    Pin("macosRunner", WORKFLOWS, r"\bmacos-(\d+)\b", "macOS runner image"),
    Pin("macosDeployment", ("docs/releasing.md",), r"defaults to macOS (\d+\.\d+)", "default macOS deployment target"),
    Pin("msvcRuntimeMin", ("core/runtime/include/wpiemu/core/launcher/jdk_policy.h",), r'kMinVcRuntime = "(\d+\.\d+)"',
        "minimum Visual C++ runtime of the JDK"),
]


def check_jdk_assets(versions: dict) -> list[str]:
    """The managed-JDK pins the app downloads: every asset must be self-consistent (the app trusts the SHA-256 only), and
    every season must name a Java major that has a download for each platform."""
    errors = []
    jdk = versions.get("jdk")
    if not isinstance(jdk, dict):
        return ["versions.json: no entry 'jdk' (the managed Java download)"]
    assets = jdk.get("assets", {})
    if not assets:
        errors.append("versions.json: jdk.assets is empty")
    majors = set()
    platforms = set()
    for name, asset in assets.items():
        where = f"versions.json jdk.assets.{name}"
        match = re.fullmatch(r"(\d+)-((?:windows|macos|linux)-(?:x64|arm64))", name)
        if not match:
            errors.append(f"{where}: the key must be <java major>-<os>-<arch>, e.g. 17-windows-x64")
        else:
            majors.add(int(match.group(1)))
            platforms.add(match.group(2))
            if not str(asset.get("version", "")).startswith(match.group(1) + "."):
                errors.append(f"{where}: version {asset.get('version')!r} is not a Java {match.group(1)} release")
        if not re.fullmatch(r"[0-9a-f]{64}", str(asset.get("sha256", ""))):
            errors.append(f"{where}: sha256 must be 64 lower-case hex digits")
        if not str(asset.get("url", "")).startswith("https://github.com/"):
            errors.append(f"{where}: url must be an https://github.com/ address (the only host the downloader allows)")
        if not str(asset.get("version", "")) or str(asset["version"]) not in str(asset.get("url", "")).replace("%2B", "+").replace("_", "."):
            errors.append(f"{where}: version {asset.get('version')!r} does not appear in the url")
        if not isinstance(asset.get("size"), int) or asset["size"] <= 0 or asset["size"] > 500 * 1024 * 1024:
            errors.append(f"{where}: size must be a positive byte count below the downloader's 500 MiB limit")
    seasons = jdk.get("seasons", {})
    if not seasons:
        errors.append("versions.json: jdk.seasons is empty (it maps a WPILib year to the Java major that season builds with)")
    for year, major in seasons.items():
        if not re.fullmatch(r"\d{4}", str(year)) or not isinstance(major, int):
            errors.append(f"versions.json jdk.seasons.{year}: expected a four-digit year mapped to a Java major number")
            continue
        for platform in sorted(platforms):
            if f"{major}-{platform}" not in assets:
                errors.append(f"versions.json jdk.seasons.{year}: Java {major} has no download for {platform}")
    return errors


def lookup(versions: dict, key: str) -> str:
    value = versions
    for part in key.split("."):
        value = value[part]
    return value


def check(root: Path, versions: dict, pins: list[Pin] = PINS) -> list[str]:
    errors = []
    for pin in pins:
        try:
            expected = lookup(versions, pin.key)
        except (KeyError, TypeError):
            errors.append(f"versions.json: no entry '{pin.key}' (needed to check {pin.what})")
            continue
        for location in pin.paths:
            files = sorted(root.glob(location))
            if not files:
                errors.append(f"{location}: no such file (checked as {pin.what}); update PINS in tools/check_versions.py")
            for file in files:
                rel = file.relative_to(root).as_posix()
                text = common.read_text(file)
                matches = list(re.finditer(pin.pattern, text, re.MULTILINE))
                if not matches:
                    errors.append(f"{rel}: no {pin.what} found; restore it or update PINS in tools/check_versions.py")
                for match in matches:
                    found = ".".join(match.groups())
                    if found != expected:
                        line = text.count("\n", 0, match.start(1)) + 1
                        errors.append(f'{rel}:{line}: {pin.what} is "{found}", but versions.json {pin.key} is "{expected}"')
    return errors


def main() -> int:
    args = common.root_parser(__doc__).parse_args()
    root = args.root.resolve()
    try:
        errors = check(root, common.load_versions(root)) + check_jdk_assets(common.load_versions(root))
    except (OSError, ValueError) as error:
        print(f"versions: cannot check repository: {error}", file=sys.stderr)
        return 2
    return common.report(errors, "versions: ok", file=sys.stderr)


if __name__ == "__main__":
    sys.exit(main())
