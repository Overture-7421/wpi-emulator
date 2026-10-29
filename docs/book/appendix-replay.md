# Appendix: replay the visible listings

The workshops are the explanation and source of truth. This helper saves transcription; it does
not download a plugin or hide an implementation in an archive. You can instead create the named
files and append their marked blocks in workshop order. Unmarked command/example blocks are
instructions, not source to append.

## 1. Export a clean practice copy

From the repository root, choose a **new**, unused destination under the ignored experiments folder:

```powershell
New-Item -ItemType Directory -Force spikes | Out-Null
git archive --format=zip -o spikes\power-up-book-replay.zip HEAD
Expand-Archive spikes\power-up-book-replay.zip spikes\power-up-book-replay
```

The export supplies the existing SDK, build files, Gradle wrappers and sample projects. The current
book pages supply all the new plugin, robot and test source. They need not be committed to be
replayed. Do not copy implementation from a previous experiment.

## 2. Adjust the registry expectation

The existing synthetic-year test expects the shipped game plus its synthetic game. Enabling our
second shipped game makes three. In the export's
[`test_new_year_plugin.cpp`](../../tests/plugins/test_new_year_plugin.cpp), replace the following
assertion group; keep the duplicate-registration assertion immediately after it:

```cpp
    REQUIRE(w.reg.plugins.size() == 3);
    REQUIRE(w.reg.plugins.findByYear(2026) != nullptr);
    REQUIRE(w.reg.plugins.findByYear(2027) != nullptr);
    REQUIRE(w.reg.plugins.findByYear(2998) != nullptr);
    CHECK(w.reg.plugins.findByYear(2026)->metadata().id == "game_2026");
    CHECK(w.reg.plugins.findByYear(2027)->metadata().id == "game_2027");
    CHECK(w.reg.plugins.findByYear(2998)->metadata().id == "game_2998");
```

This is not weakening the test: it still asserts the exact count and every expected identity.
The helper below applies this same narrow edit automatically; do not make it by hand first if
using the helper.

## 3. Optional assembler

Save this script outside the export, for example as <code>spikes\assemble_power_up_book.py</code>.
Run it from the repository root, passing the current book folder and the **fresh export**:

```powershell
python spikes\assemble_power_up_book.py docs\book spikes\power-up-book-replay
```

The script reads only marked blocks, groups repeated destinations in reading order, checks paths
and refuses an already assembled or Git-root destination. It also checks the old registry
assertions before changing them, so an incompatible baseline fails explicitly. Nothing is written
until those checks pass.

```python
import argparse
from pathlib import Path, PureWindowsPath
import re

parser = argparse.ArgumentParser(description="Assemble visible Power Up workshop listings")
parser.add_argument("book", type=Path)
parser.add_argument("export", type=Path)
args = parser.parse_args()
root = args.export.resolve()
if not (root / "CMakeLists.txt").is_file() or (root / ".git").exists():
    raise SystemExit("Use a fresh source export, not a Git checkout")
if (root / "seasons" / "2027" / "game").exists():
    raise SystemExit("Destination already contains the game; choose a fresh export")

pages = (
    "02a-field-implementation.md",
    "03a-reset-and-pieces.md",
    "04a-match-and-snapshots.md",
    "04b-zone-interactions.md",
    "04c-module-and-registration.md",
    "05a-driven-robot.md",
    "05b-physics-tests.md",
    "05c-full-match.md",
)
pattern = re.compile(
    r"<!-- book-file: ([^\r\n]+) -->\r?\n```[^\r\n]*\r?\n(.*?)\r?\n```",
    re.DOTALL,
)
files = {}
for page in pages:
    text = (args.book / page).read_text(encoding="utf-8")
    for name, code in pattern.findall(text):
        relative = PureWindowsPath(name)
        if relative.is_absolute() or relative.drive or ".." in relative.parts:
            raise SystemExit(f"Unsafe destination: {name}")
        destination = root.joinpath(*relative.parts).resolve()
        if not destination.is_relative_to(root):
            raise SystemExit(f"Destination escapes export: {name}")
        files.setdefault(destination, []).append(code + "\n")
if len(files) != 11:
    raise SystemExit(f"Expected 11 complete source/model files, found {len(files)}")

registry = root / "tests" / "plugins" / "test_new_year_plugin.cpp"
original = registry.read_text(encoding="utf-8")
updated = original
edits = (
    ("REQUIRE(w.reg.plugins.size() == 2);", "REQUIRE(w.reg.plugins.size() == 3);"),
    (
        "    REQUIRE(w.reg.plugins.findByYear(2026) != nullptr);",
        "    REQUIRE(w.reg.plugins.findByYear(2026) != nullptr);\n"
        "    REQUIRE(w.reg.plugins.findByYear(2027) != nullptr);",
    ),
    (
        '    CHECK(w.reg.plugins.findByYear(2026)->metadata().id == "game_2026");',
        '    CHECK(w.reg.plugins.findByYear(2026)->metadata().id == "game_2026");\n'
        '    CHECK(w.reg.plugins.findByYear(2027)->metadata().id == "game_2027");',
    ),
)
for old, new in edits:
    if updated.count(old) != 1:
        raise SystemExit("Registry baseline differs; inspect it before applying this lesson")
    updated = updated.replace(old, new, 1)

for destination, chunks in files.items():
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("\n".join(chunks), encoding="utf-8", newline="\n")
    print(destination.relative_to(root))
registry.write_text(updated, encoding="utf-8", newline="\n")
print("Assembled 11 files and updated the registry assertions.")
```

This is a convenience tool for this book's fixed baseline, not a general Markdown execution engine.
Read the source blocks before running them, as you would any tutorial code.

## 4. Build and verify the assembled result

Work from the export root, not the original checkout. Follow the
[plugin build checkpoint](04c-module-and-registration.md#4-first-complete-build-checkpoint), then
the [physics test command](05b-physics-tests.md#5-run-the-focused-proof) and the
[real-HAL match command](05c-full-match.md#5-run-it-and-read-the-evidence).
Also run the registry test and repository checks:

```powershell
ctest --test-dir build\power-up-book -C Release `
  -R "^a new season registers beside the built-ins" --output-on-failure
python tools\check_boundaries.py
python tools\check_seasons.py
python tools\check_versions.py
```

Run `python tools\check_docs.py` from the original **Git checkout** after editing the book.
That check uses Git to enumerate tracked and unignored files; an archive is not a Git checkout
and cannot provide its own input list. The checker validates links, not C++ behavior. It also cannot prove a rendered
field looks right: use the actual desktop inspection in the match workshop.

If dependency sources are already cached, follow [Building](../BUILDING.md) to reuse their
**source** directories. Keep this export's generated dependency binaries and CMake cache separate.
The published build command permits downloads for a reader without a cache.

## Verification record

The visible listings were assembled into a **fresh source export**, not copied from the earlier
prototype. MSVC 19.51 / Visual Studio 2026, Ninja Debug and warnings-as-errors built the plugin,
aggregator, unit tests, host match executable and real 2027 HALSim extension. Only dependency
sources and installed WPILib artifacts were reused; generated binaries belonged to the new export.

| Check on the replay | Measured result |
|---|---|
| Real-physics cases and exact registry identities | All six CTest cases passed |
| Java robot, published saved model, real HALSim | Auto 15.290 / final 150.290; reset repeated delivery |
| C++ robot, published saved model, real HALSim | Auto 15.290 / final 150.290; reset repeated delivery |
| Combined robot-process test | 970 assertions passed; one selected case actually ran |
| Match timing and allocation | 15 s auto, 2 s disabled gap, 135 s teleop, 3 s final disabled; 83 bodies |
| Boundary, season and version checks | Passed |

Both robot projects compiled during those real launches. The test read the saved JSON published in
the book, including its omitted-field defaults, rather than constructing a substitute robot model.
Both robots moved from X=1.0 m to approximately X=3.172 m; no test teleported their delivered cube
or injected a score. The matcher intentionally deselected the separate vision test; that is not
a failure or coverage of vision.

The same plugin implementation was previously inspected in the real Qt Quick 3D desktop: the
corrected rows, pyramids and off-field supply were visible. The fresh replay above was Qt-free;
it did **not** repeat that visual inspection. No GCC/Linux or full-suite claim is made here, and
none of this tests the unimplemented full-game features.
