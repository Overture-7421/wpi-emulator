"""Helpers shared by the repository's Python tools; standard library only.

Tools import this as `common`. Running a script puts tools/ on sys.path from any working directory,
and tools/tests/support.py does the same for the unit tests.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


def read_text(path: Path, errors: str = "strict") -> str:
    """UTF-8 text of a file; pass errors="replace" to scan files that may hold stray bytes."""
    return path.read_text(encoding="utf-8", errors=errors)


def write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def load_versions(root: Path = REPO_ROOT) -> dict:
    """The entries of <root>/versions.json, the single home of the repository's version pins."""
    return json.loads(read_text(root / "versions.json"))


def load_seasons(root: Path = REPO_ROOT) -> dict[str, dict]:
    """The season manifests of <root>/seasons/<year>/season.json, keyed by year, in ascending year order."""
    found = {}
    for manifest in sorted((root / "seasons").glob("*/season.json")):
        if not manifest.parent.name.startswith("_"):   # seasons/_template is a skeleton, not a season
            found[manifest.parent.name] = json.loads(read_text(manifest))
    return found


def load_abis(root: Path = REPO_ROOT) -> dict[str, dict]:
    """The HAL ABI profiles of <root>/seasons/abi/<id>.json, keyed by id."""
    return {path.stem: json.loads(read_text(path)) for path in sorted((root / "seasons" / "abi").glob("*.json"))}


def root_parser(description: str, **options) -> argparse.ArgumentParser:
    """Argument parser with the --root option every repository check takes."""
    parser = argparse.ArgumentParser(description=description, **options)
    parser.add_argument("--root", type=Path, default=REPO_ROOT)
    return parser


def report(errors: list[str], ok: str, *, heading: str | None = None, indent: str = "", file=None) -> int:
    """Print a check's findings, one per line, and return its exit status: 1 with findings, else print `ok` and return 0."""
    if errors:
        out = sys.stdout if file is None else file
        if heading:
            print(heading, file=out)
        for error in errors:
            print(indent + error, file=out)
        return 1
    print(ok)
    return 0
