#!/usr/bin/env python3
"""Check every Mach-O slice of the app and extension against the bundle's declared minimum."""
import plistlib
import re
import subprocess
import sys
from pathlib import Path


def version(value: str) -> tuple[int, ...]:
    parts = tuple(int(part) for part in value.split("."))
    return parts + (0,) * max(0, 3 - len(parts))


def main() -> None:
    bundle = Path(sys.argv[1])
    with (bundle / "Contents" / "Info.plist").open("rb") as stream:
        minimum = plistlib.load(stream)["LSMinimumSystemVersion"]
    for relative in ("MacOS/wpiemu_app", "PlugIns/wpiemu/libhalsim_wpiemu.dylib"):
        binary = bundle / "Contents" / relative
        output = subprocess.check_output(["otool", "-arch", "all", "-l", str(binary)], text=True)
        values = re.findall(r"\bminos\s+([\d.]+)", output)
        values += re.findall(r"cmd LC_VERSION_MIN_MACOSX\b.*?\bversion\s+([\d.]+)",
                             output, flags=re.DOTALL)
        if not values or any(version(value) > version(minimum) for value in values):
            raise SystemExit(f"{binary}: Mach-O minima {values} exceed or cannot verify "
                             f"LSMinimumSystemVersion {minimum}")
        print(f"minimum OS: {binary.name}: {values} <= {minimum}")


if __name__ == "__main__":
    main()
