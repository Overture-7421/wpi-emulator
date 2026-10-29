# Appendix B. Reproduction

## B.1 Baseline and prerequisites

The exact integration edits target repository commit <code>103a8ea</code>.
They are not fuzzy patches. A later source revision requires reviewing their
context rather than disabling the match check.

The native environment needs Git, Python 3.10+, CMake 3.25+, Ninja and a
compiler supporting the selected HAL profile. The systemcore preview requires
C++23 toolchain support: the tested Windows compiler is Visual Studio 2026.
The plugin's own source uses C++20. JDK 25 and the season's pinned WPILib
desktop artifacts are required for real-HAL tests.

Qt is unnecessary for unit or robot-process tests. The desktop additionally
needs the matching Qt 6.8.3 MSVC kit, including Quick 3D, Shader Tools and
Multimedia. [Building](../BUILDING.md) owns tool installation and cache options.

## B.2 Disposable source tree

Run these commands from the original repository root. The destination must
be new; this avoids overwriting an existing working tree.

```powershell
New-Item -ItemType Directory spikes\book-work
git archive --format=tar --output=spikes\book-base.tar 103a8ea
tar -xf spikes\book-base.tar -C spikes\book-work
Remove-Item -LiteralPath spikes\book-base.tar
$repository = (Get-Location).Path
```

Save Listing B.1 as <code>spikes\tangle_book.py</code>. Then extract the complete
book into the export:

```powershell
python spikes\tangle_book.py --book "$repository\docs\book" --root "$repository\spikes\book-work" --through 22
Push-Location spikes\book-work
python seasons\2027\game\tools\copy_robots.py
python seasons\2027\game\tools\generate_sounds.py
```

No plugin file is copied from a prototype directory. The scaffold helper supplies
only unchanged Gradle files from the exported season samples.

## B.3 Extraction semantics

A `book-file` marker names a complete file. Later revisions replace that file
in full. A `book-edit` marker contains JSON with one old string and one new
string; the old string must occur exactly once. Edits are applied to the
pending content in chapter order.

All paths must stay inside the export. Absolute paths, parent traversal,
reserved output locations and symlink escapes are rejected. The tool prepares
the whole selection before writing, so a failed exact edit does not leave
a partially applied selection.

The tool reads numbered chapters through the selected checkpoint. At Chapter
22 it also includes Appendix E's worked tests. The tool's own listing is not
part of the plugin and is copied manually to avoid a bootstrap dependency.

Listing B.1. Complete optional extraction tool.

```python
"""Reproduce the book's complete-file listings in a disposable repository export."""
import argparse
import json
from pathlib import Path, PureWindowsPath
import re

MARKER = re.compile(
    r"<!-- book-(file|edit): ([^\r\n]+?) -->\s*\n```[^\r\n]*\n(.*?)\n```",
    re.DOTALL,
)


def target(root, name):
    windows = PureWindowsPath(name)
    if windows.drive or windows.root or ".." in windows.parts:
        raise ValueError(f"unsafe listing path: {name}")
    parts = name.replace("\\", "/").split("/")
    if any(p in ("", ".", ".git", "build", "spikes") for p in parts):
        raise ValueError(f"reserved listing path: {name}")
    path = root.joinpath(*parts).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f"listing escapes export: {name}")
    return path


def assemble(book, root, through=22):
    if not root.is_dir():
        raise ValueError("the destination must be an existing disposable export")
    if (root / ".git").exists():
        raise ValueError("the destination must be an export, not a Git working tree")
    pending = {}
    seen = set()
    chapters = sorted(book.glob("[0-9][0-9]-*.md"))
    if through >= 22 and (book / "appendix-e-solutions.md").is_file():
        chapters.append(book / "appendix-e-solutions.md")
    for chapter in chapters:
        if chapter.name[:2].isdigit() and int(chapter.name[:2]) > through:
            continue
        text = chapter.read_text(encoding="utf-8")
        for kind, name, content in MARKER.findall(text):
            path = target(root, name)
            if kind == "file":
                pending[path] = content + "\n"
            else:
                edit = json.loads(content)
                if set(edit) != {"old", "new"} or not edit["old"]:
                    raise ValueError(f"invalid edit in {chapter.name}: {name}")
                before = pending.get(path)
                if before is None:
                    before = path.read_text(encoding="utf-8")
                if before.count(edit["old"]) != 1:
                    raise ValueError(f"edit must match exactly once: {name}")
                pending[path] = before.replace(edit["old"], edit["new"], 1)
            seen.add(path)
    if not seen:
        raise ValueError("no listings selected")
    for path, content in pending.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists() or path.read_text(encoding="utf-8") != content:
            path.write_text(content, encoding="utf-8", newline="\n")
    return len(seen)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--book", type=Path, required=True)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--through", type=int, default=22, choices=range(5, 23))
    args = parser.parse_args()
    print(f"Wrote {assemble(args.book, args.root, args.through)} files")


if __name__ == "__main__":
    main()
```

## B.4 Native configure and tests

Adjust the Visual Studio installation path for the installed edition. Keep
environment setup and the build in one process. This example allows downloads
of missing pinned dependencies:

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2027_alpha7\jdk'
cmd.exe /c 'call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul && cmake -S . -B build\book -G Ninja -DCMAKE_BUILD_TYPE=Release -DWPIEMU_BUILD_APP=OFF -DWPIEMU_WARNINGS_AS_ERRORS=ON -DWPIEMU_WPILIB_SEASON=2027 -DWPIEMU_SEASONS=2027 -DWPIEMU_WPILIB_DOWNLOAD=ON -DWPIEMU_REQUIRE_REAL_HAL=ON && cmake --build build\book --target wpiemu_tests wpiemu_power_up_e2e_tests wpiemu_season_e2e_tests --parallel 8'
ctest --test-dir build\book -R "Power UP:" --no-tests=error --output-on-failure
.\build\book\bin\wpiemu_tests.exe '--source-file=*test_rules*'
python tools\check_boundaries.py
python tools\check_seasons.py
python tools\check_versions.py
```

The pure-rule selector must report a nonzero test count. A successful command
that selects no test is not evidence.

For each chapter checkpoint, begin with a new baseline export, use
<code>--through N</code>, build <code>wpiemu_tests</code> and run the chapter's
selector. Chapters 13 onward also need the scaffold helper; Chapter 20 onward
needs the sound generator. Starting from a later checkpoint and extracting
backward would leave future test files behind and is not a valid replay.

Chapters 5-11 test registration and progressively independent components.
Chapter 12 adds state and synchronous-session tests. Chapter 14 adds the host
match target; Chapters 15-19 refine rules and extend the same tests.

## B.5 Real robot processes

The opt-in test builds temporary copies of the published projects. Their
models are read through the normal configuration pipeline. No separate
hand-written model is hidden in the harness.

```powershell
$env:WPIEMU_E2E = '1'
$env:WPIEMU_E2E_JAVA_HOME = $env:JAVA_HOME
ctest --test-dir build\book -R "e2e.power_up_" --no-tests=error --output-on-failure
ctest --test-dir build\book -R "^e2e.season_" --no-tests=error --output-on-failure
```

The additional cases exercise vault/Force and anchored climbing in both
languages, plus a C++ match with virtual partners and a physical opponent
cube. The original season probe runs its original projects separately.
Missing prerequisites produce a skipped test, not a passed match.

## B.6 Desktop inspection

Generate the tone before configuring. From the same Visual Studio environment,
configure the existing build with <code>WPIEMU_BUILD_APP=ON</code> and
<code>CMAKE_PREFIX_PATH=C:\Qt\6.8.3\msvc2022_64</code>, then build
<code>wpiemu_app</code>. The Qt kit's binary directory must be on `PATH`
when launching an undeployed build.

```powershell
$env:PATH = 'C:\Qt\6.8.3\msvc2022_64\bin;' + $env:PATH
.\build\book\bin\wpiemu_app.exe --project samples\season_2027\power_up_cpp
```

Inspect the saved model before starting. Select Blue station 1. In Driver
Station settings, set the Practice durations to 15 s autonomous, 2 s gap and
135 s teleoperation, rather than relying on saved defaults. Use the following
observation protocol:

1. Before enable, inspect six fence cubes and the 6/3/1 pyramid at each switch.
   Portal reserves must be outside the carpet, not a central grid.
2. Read the assignment and check that the start lane faces its home plate.
3. During autonomous, observe forward drive, stopping, release and plate motion.
   The visual plate and cube collision must remain aligned.
4. Disable and check that the score freezes while physics may settle.
5. Reset and verify original staging, inventory and score.
6. With audio enabled, listen for a single ownership cue rather than a repeating
   sound; inspect the Scenario editor panel and its default settings.

This protocol is distinct from an offscreen UI test. Appendix C records which
observations were completed for the published reproduction.

## B.7 Diagnostics

| Symptom | First check |
|---|---|
| Exact edit does not match | Baseline commit and whether the export was already modified |
| No match test registered | Scaffold helper ran before CMake configuration |
| Stub extension or missing HAL | Season selection and complete platform-specific WPILib artifacts |
| Gradle rejects Java version | JDK 25 path for the preview |
| Motors declared but no movement | Device mapping, enabled mode and activation threshold |
| Release without expected velocity | Body enabled before pose/velocity assignment |
| Correct collisions but axis-aligned cubes | Entity-group orientation limit, F-25 |
| Scene blank in an offscreen run | Use a real Quick 3D window for L5 |

Return to the original repository with `Pop-Location` after the experiment.
Adopting the plugin uses the same listings and integration edits in a development
branch; no core or HAL ABI implementation change is required.

[Appendix C](appendix-c-conformance.md)
