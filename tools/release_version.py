#!/usr/bin/env python3
"""Version handling of the release workflow (docs/releasing.md#release-automation-and-publication).

The committed `project(VERSION ...)` of CMakeLists.txt is the placeholder 0.0.0, the app's development-build marker.
A release overrides it in the build workspace only, so installers and the app carry the version typed into the workflow.

  release_version.py resolve VERSION [--tags-file FILE] [--github-output]   validate and normalize a release version
  release_version.py [--root DIR] apply VERSION                             rewrite the project version in CMakeLists.txt
  release_version.py artifacts DIRECTORY VERSION                            installers carry VERSION and have a checksum
"""
from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path
from typing import NamedTuple

import common

RELEASE = re.compile(r"^v?(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)(?:-([0-9A-Za-z][0-9A-Za-z.-]*))?$")
PROJECT_VERSION = re.compile(r"(^project\([^)]*?\bVERSION\s+)(\d+(?:\.\d+){0,3})", re.MULTILINE)
INSTALLER_SUFFIXES = {".exe", ".zip", ".dmg", ".AppImage"}
DEVELOPMENT_VERSION = "0.0.0"


class Release(NamedTuple):
    tag: str          # v0.0.8, or v0.0.8-rc1
    version: str      # 0.0.8: what CMake accepts and what the packages are named after
    prerelease: bool  # a "-suffix" marks a GitHub prerelease


def parse_release(text: str) -> Release:
    match = RELEASE.match(text.strip())
    if not match:
        raise ValueError(f'"{text}" is not a release version: use vMAJOR.MINOR.PATCH, optionally followed by -suffix (v0.0.8, v1.2.0-rc1)')
    version = ".".join(match.group(1, 2, 3))
    if version == DEVELOPMENT_VERSION:
        raise ValueError(f"{DEVELOPMENT_VERSION} is the development-build marker: the app never checks for updates with it")
    suffix = match.group(4)
    return Release("v" + version + (f"-{suffix}" if suffix else ""), version, suffix is not None)


def _base(tag: str) -> tuple[int, int, int] | None:
    match = RELEASE.match(tag)
    return tuple(int(part) for part in match.group(1, 2, 3)) if match else None


def check_release(release: Release, existing_tags: list[str]) -> list[str]:
    """Reasons not to release `release` given the repository's tags; tags that are not vX.Y.Z[-suffix] are ignored."""
    errors = []
    if release.tag in existing_tags:
        errors.append(f"tag {release.tag} already exists")
    wanted = _base(release.tag)
    known = [(_base(tag), tag) for tag in existing_tags if tag.startswith("v") and _base(tag)]
    if known:
        newest, newest_tag = max(known)
        if wanted < newest:
            errors.append(f"{release.tag} is lower than the existing tag {newest_tag}")
    if release.prerelease:
        finals = [tag for base, tag in known if base == wanted and "-" not in tag]
        if finals:
            errors.append(f"{release.tag} is a prerelease of {finals[0]}, which is already released")
    return errors


def set_project_version(cmake_text: str, version: str) -> str:
    """`cmake_text` with the VERSION of its project() call replaced; ValueError unless there is exactly one."""
    found = PROJECT_VERSION.findall(cmake_text)
    if len(found) != 1:
        raise ValueError(f"expected one project(... VERSION x.y.z) call, found {len(found)}")
    return PROJECT_VERSION.sub(lambda match: match.group(1) + version, cmake_text)


def read_project_version(cmake_text: str) -> str:
    match = PROJECT_VERSION.search(cmake_text)
    if not match:
        raise ValueError("no project(... VERSION x.y.z) call found")
    return match.group(2)


def artifact_errors(directory: Path, version: str) -> list[str]:
    prefix = f"wpi-emulator-{version}-"
    installers = sorted(path for path in directory.iterdir() if path.is_file() and path.suffix in INSTALLER_SUFFIXES)
    if not installers:
        return [f"{directory}: no installers found"]
    errors = []
    for path in installers:
        if not path.name.startswith(prefix):
            errors.append(f"{path.name}: not named {prefix}*, so the build did not use version {version}")
        if not path.with_name(path.name + ".sha256").is_file():
            errors.append(f"{path.name}: no .sha256 sidecar")
    return errors


def _fail(message: str) -> int:
    print(f"::error::{message}" if os.environ.get("GITHUB_ACTIONS") else f"error: {message}", file=sys.stderr)
    return 1


def _resolve(args: argparse.Namespace) -> int:
    try:
        release = parse_release(args.version)
    except ValueError as error:
        return _fail(str(error))
    tags = common.read_text(args.tags_file).split() if args.tags_file else []
    errors = check_release(release, tags)
    if errors:
        return _fail("; ".join(errors))
    lines = [f"tag={release.tag}", f"version={release.version}", f"prerelease={str(release.prerelease).lower()}"]
    print("\n".join(lines))
    if args.github_output:
        with open(os.environ["GITHUB_OUTPUT"], "a", encoding="utf-8") as stream:
            stream.write("\n".join(lines) + "\n")
    return 0


def _apply(args: argparse.Namespace) -> int:
    path = args.root / "CMakeLists.txt"
    try:
        release = parse_release(args.version)
        text = common.read_text(path)
        before = read_project_version(text)
        common.write_text(path, set_project_version(text, release.version))
    except (OSError, ValueError) as error:
        return _fail(f"{path}: {error}")
    print(f"{path}: project version {before} -> {release.version}")
    return 0


def _artifacts(args: argparse.Namespace) -> int:
    try:
        errors = artifact_errors(args.directory, args.version)
    except OSError as error:
        return _fail(str(error))
    return common.report(errors, f"artifacts: all named for version {args.version}", file=sys.stderr)


def main() -> int:
    parser = common.root_parser(__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)
    resolve = commands.add_parser("resolve", help="validate and normalize a release version")
    resolve.add_argument("version")
    resolve.add_argument("--tags-file", type=Path, help="existing tags, whitespace separated")
    resolve.add_argument("--github-output", action="store_true", help="also append the results to $GITHUB_OUTPUT")
    resolve.set_defaults(run=_resolve)
    apply = commands.add_parser("apply", help="rewrite the project version in <root>/CMakeLists.txt")
    apply.add_argument("version")
    apply.set_defaults(run=_apply)
    artifacts = commands.add_parser("artifacts", help="check installer names and checksum sidecars")
    artifacts.add_argument("directory", type=Path)
    artifacts.add_argument("version", help="numeric version, e.g. 0.0.8")
    artifacts.set_defaults(run=_artifacts)
    args = parser.parse_args()
    args.root = args.root.resolve()
    return args.run(args)


if __name__ == "__main__":
    sys.exit(main())
