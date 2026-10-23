#!/usr/bin/env python3
"""Check the season packs (docs/reference/seasons.md, ADR 0018).

  * seasons/<year>/season.json and seasons/abi/<id>.json are well formed and consistent: exactly one default
    season, a pinned WPILib version that is also verified, a known HAL ABI and launch profile, existing samples
    whose GradleRIO version matches their season, and a game plugin that exists when the manifest names one.
  * Season-year literals (and WPIEMU_WPILIB_YEAR) appear only in seasons/, samples, tests and docs. Everything else
    reads season facts from the manifests. tools/season_literals.allow lists the shrink-only exceptions (path = count);
    a count that no longer matches must be lowered, so the list can only shrink.
Run:  check_seasons.py [--root DIR]
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import common

LAUNCH_PROFILES = {"gradlerio-classic", "gradlerio-modern"}
CHANNELS = {"stable", "preview"}
SCHEMAS = {"photon-us", "photon-ns"}
GRADLERIO = re.compile(r'(?:edu\.wpi\.first|org\.wpilib)\.GradleRIO"\s+version\s+"([^"]+)"')

SCANNED = {".cpp", ".h", ".hpp", ".cc", ".mm", ".cmake", ".txt", ".py", ".yml", ".yaml", ".sh", ".ps1", ".in", ".groovy",
           ".qml", ".gradle"}
CPP = {".cpp", ".h", ".hpp", ".cc", ".mm", ".qml"}
EXEMPT_TOP = {"seasons", "samples", "docs", "tests", "spikes", "build", "out", "node_modules"}
# Checker sources that name fixture years on purpose.
EXEMPT_FILES = {"tools/check_seasons.py", "tools/check_boundaries.py"}
EXEMPT_PARTS = {"tests", "testing", "_deps", ".git"}
# Product names that merely contain a year ("Visual Studio 18 2026", "windows-2025-vs2026") are not season literals.
PRODUCT = re.compile(r"Visual Studio(?: \d+)? \d{4}|(?:vs|msvc)\d{4}|VisualStudio\.\d{4}", re.IGNORECASE)
ALLOWLIST = "tools/season_literals.allow"
MARKER = "season-literal-ok:"


def code_of(text: str, suffix: str) -> list[tuple[int, str]]:
    """(line number, code) pairs with comments removed; a line carrying the marker is skipped."""
    if suffix in CPP:
        text = re.sub(r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), text, flags=re.S)
        comment = re.compile(r"//.*$")
    else:
        comment = re.compile(r"(?<![\"'\w$])#.*$")
    out = []
    for number, line in enumerate(text.splitlines(), 1):
        if MARKER in line:
            continue
        out.append((number, comment.sub("", line)))
    return out


def literal_pattern(years: list[str]) -> re.Pattern:
    """The year of any season pack as a whole word, or the legacy WPIEMU_WPILIB_YEAR definition."""
    return re.compile(r"\b(?:" + "|".join(re.escape(y) for y in years) + r")\b|WPIEMU_WPILIB_YEAR")


def scan_literals(root: Path, files: list[str], years: list[str]) -> dict[str, int]:
    pattern = literal_pattern(years)
    found: dict[str, int] = {}
    for rel in files:
        parts = rel.split("/")
        if parts[0] in EXEMPT_TOP or EXEMPT_PARTS.intersection(parts[:-1]) or rel == ALLOWLIST:
            continue
        suffix = Path(rel).suffix
        if suffix not in SCANNED or rel in EXEMPT_FILES:
            continue
        try:
            text = common.read_text(root / rel, "replace")
        except OSError:
            continue
        count = sum(len(pattern.findall(PRODUCT.sub("", code))) for _, code in code_of(text, suffix))
        if count:
            found[rel] = count
    return found


def read_allowlist(root: Path) -> dict[str, int]:
    path = root / ALLOWLIST
    if not path.exists():
        return {}
    allowed = {}
    for line in common.read_text(path).splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            name, _, count = line.rpartition("=")
            allowed[name.strip()] = int(count)
    return allowed


def literal_errors(root: Path, files: list[str], years: list[str]) -> list[str]:
    errors = []
    found = scan_literals(root, files, years)
    allowed = read_allowlist(root)
    for rel, count in sorted(found.items()):
        if rel not in allowed:
            errors.append(f"{rel}: {count} season-year literal(s); read season facts from seasons/*/season.json "
                          f"(or mark a line '{MARKER} <reason>')")
        elif count > allowed[rel]:
            errors.append(f"{rel}: {count} season-year literal(s), allowlist permits {allowed[rel]}")
        elif count < allowed[rel]:
            errors.append(f"{rel}: only {count} season-year literal(s) remain; lower its {ALLOWLIST} count from {allowed[rel]}")
    for rel in sorted(set(allowed) - set(found)):
        errors.append(f"{ALLOWLIST}: {rel} no longer has season-year literals; remove the entry")
    return errors


def manifest_errors(root: Path, seasons: dict[str, dict], abis: dict[str, dict]) -> list[str]:
    errors = []
    if not seasons:
        return ["seasons: no seasons/<year>/season.json found"]
    defaults = [year for year, season in seasons.items() if season.get("default") is True]
    if len(defaults) != 1:
        errors.append(f"seasons: exactly one season must set \"default\": true (found {defaults or 'none'})")
    for name, abi in abis.items():
        where = f"seasons/abi/{name}.json"
        if abi.get("id") != name:
            errors.append(f"{where}: id must be {name!r}")
        if not isinstance(abi.get("cxxStandard"), int):
            errors.append(f"{where}: cxxStandard must be an integer")
        for key in ("hal", "wpiutil", "ntcore", "wpinet"):
            if not abi.get("groups", {}).get(key):
                errors.append(f"{where}: groups.{key} is missing")
        for key in ("hal", "wpiutil", "ntcore"):
            if not abi.get("headers", {}).get(key):
                errors.append(f"{where}: headers.{key} is missing")
    sample_owner: dict[str, str] = {}
    for year, season in seasons.items():
        where = f"seasons/{year}/season.json"
        if not re.fullmatch(r"\d{4}", year) or season.get("year") != int(year if year.isdigit() else 0):
            errors.append(f"{where}: year must equal the folder name")
        if season.get("channel") not in CHANNELS:
            errors.append(f"{where}: channel must be one of {sorted(CHANNELS)}")
        wpilib = season.get("wpilib", {})
        version = str(wpilib.get("version", ""))
        if not version.startswith(year + "."):
            errors.append(f"{where}: wpilib.version must begin with {year}.")
        verified = wpilib.get("verified", [])
        if version not in verified:
            errors.append(f"{where}: wpilib.version {version!r} must be listed in wpilib.verified")
        if not str(wpilib.get("installFolder", "")).startswith(year):
            errors.append(f"{where}: wpilib.installFolder must begin with {year}")
        platform = season.get("platform", {})
        if platform.get("abi") not in abis:
            errors.append(f"{where}: platform.abi {platform.get('abi')!r} has no seasons/abi/<id>.json")
        if platform.get("launch") not in LAUNCH_PROFILES:
            errors.append(f"{where}: platform.launch must be one of {sorted(LAUNCH_PROFILES)}")
        if not isinstance(season.get("toolchain", {}).get("javaMajor"), int):
            errors.append(f"{where}: toolchain.javaMajor must be an integer")
        for vendor, entry in season.get("vendors", {}).items():
            if entry.get("schema") not in SCHEMAS:
                errors.append(f"{where}: vendors.{vendor}.schema must be one of {sorted(SCHEMAS)}")
            if entry.get("defaultVersion", "").lstrip("v") not in [v.lstrip("v") for v in entry.get("verified", [])]:
                errors.append(f"{where}: vendors.{vendor}.defaultVersion must be listed in verified")
        game = season.get("game")
        if game is not None and game.get("plugin") != f"game_{year}":
            errors.append(f"{where}: game.plugin must be game_{year} (the plugin id equals its folder and namespace)")
        if game is not None and not (root / "seasons" / year / "game").is_dir():
            errors.append(f"{where}: game.plugin {game.get('plugin')!r} needs the directory seasons/{year}/game")
        if game is None and (root / "seasons" / year / "game").is_dir():
            errors.append(f"{where}: seasons/{year}/game exists but the manifest has \"game\": null")
        if "probes" in season and not (root / season["probes"]).is_dir():
            errors.append(f"{where}: probes folder {season['probes']} does not exist")
        for sample in season.get("samples", []):
            if sample in sample_owner:
                errors.append(f"{where}: sample {sample} is also listed by season {sample_owner[sample]}")
            sample_owner[sample] = year
            gradle = root / sample / "build.gradle"
            if not gradle.is_file():
                errors.append(f"{where}: sample {sample} has no build.gradle")
                continue
            found = [m.group(1) for m in GRADLERIO.finditer(common.read_text(gradle))]
            if not found:
                errors.append(f"{sample}/build.gradle: no literal GradleRIO version found")
            for value in found:
                if value != version:
                    errors.append(f'{sample}/build.gradle: GradleRIO version is "{value}", but {where} wpilib.version is "{version}"')
    for gradle in sorted(root.glob("samples/*/*/build.gradle")):
        sample = gradle.parent.relative_to(root).as_posix()
        if sample not in sample_owner:
            errors.append(f"{sample}: not listed in the samples of any season manifest")
    return errors


def tracked_files(root: Path) -> list[str]:
    import subprocess
    try:
        out = subprocess.run(["git", "-C", str(root), "ls-files", "--cached", "--others", "--exclude-standard"],
                             capture_output=True, text=True, check=True).stdout
        return [line for line in out.splitlines() if (root / line).is_file()]
    except (OSError, subprocess.CalledProcessError):
        return [p.relative_to(root).as_posix() for p in root.rglob("*") if p.is_file() and ".git" not in p.parts]


def check(root: Path, files: list[str] | None = None) -> list[str]:
    seasons = common.load_seasons(root)
    errors = manifest_errors(root, seasons, common.load_abis(root))
    return errors + literal_errors(root, tracked_files(root) if files is None else files, list(seasons))


def main() -> int:
    args = common.root_parser(__doc__).parse_args()
    try:
        errors = check(args.root.resolve())
    except (OSError, ValueError) as error:
        print(f"seasons: cannot check repository: {error}", file=sys.stderr)
        return 2
    return common.report(errors, "seasons: season packs consistent", heading="Season pack problems:", indent="  ")


if __name__ == "__main__":
    sys.exit(main())
