#!/usr/bin/env python3
"""Refresh and verify SHA256 sidecars after all release-artifact mutations."""
import argparse
import hashlib
from pathlib import Path

import common


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--verify", action="store_true", help="verify only, without rewriting sidecars")
    args = parser.parse_args()
    artifacts = sorted(path for path in args.directory.iterdir()
                       if path.is_file() and path.suffix in {".exe", ".zip", ".dmg", ".AppImage"})
    if not artifacts:
        parser.error("no release artifacts found")
    for artifact in artifacts:
        sidecar = artifact.with_name(artifact.name + ".sha256")
        expected = f"{digest(artifact)}  {artifact.name}\n"
        if not args.verify:
            sidecar.write_bytes(expected.encode("utf-8"))
        if common.read_text(sidecar) != expected:
            raise SystemExit(f"checksum mismatch: {artifact}")
        print(f"checksum: {artifact.name}: ok")


if __name__ == "__main__":
    main()
