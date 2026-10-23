# Tutorial: make your first change

Read this when you want to build the project, make a small code change, and test it.
This exercise temporarily adds a clearly named copy of an existing motor preset.
It is practice data, not a real motor model; remove it before finishing.

## Build and run a baseline test

From the repository root, follow the Windows Visual Studio steps in
[Build and test](../BUILDING.md#quick-start-on-windows). Then run the existing motor
catalog test:

```powershell
ctest --preset windows-vs2022-release -R "^every catalog motor reproduces its stall and free operating points$" --output-on-failure
```

Expected result: CTest finds one test and reports it passed. This test checks the
operating points for every motor in the built-in catalog. It does not test the editor
UI or a robot project.

Before editing, inspect `git status --short`. Start only when you can tell your work
from any changes already in the checkout. Do not overwrite, reset, or discard someone
else's changes.

## Add a temporary preset and run its existing test

The SDK owns the built-in motor table in
[`motor_model.h`](../../sdk/include/wpiemu/sdk/dynamics/motor_model.h). Find the NEO row
in `motors::presets()` and add this temporary row next to it:

```cpp
row("practice_neo", "NEO (practice copy)", 12, 2.6, 105, 1.8, 5676),
```

This copies the existing NEO values only to demonstrate how data reaches the catalog
and editor choices. It is not a second manufacturer motor and must not be committed.
Real motor data needs a source and independent expected-value assertions; see
[the motor and mechanisms guide](../guides/mechanisms.md).

The existing focused test in
[`test_linear_system_and_motors.cpp`](../../tests/mechanisms/test_linear_system_and_motors.cpp)
iterates over every catalog entry and checks stall/free motor operating points. It
includes this temporary row without adding a test case or inventing a new CTest name.

Build the project and run only that existing case:

```powershell
cmake --build --preset windows-vs2022-release --parallel
ctest --preset windows-vs2022-release -R "^every catalog motor reproduces its stall and free operating points$" --output-on-failure
```

Expected result: the build succeeds, CTest finds one test, and it passes. The test
proves the catalog's motor model reproduces each row's specified stall and free
operating points. It does not prove a new motor behaves correctly under load.

## Inspect the editor choice

The editor demo uses a fake backend and does not save a robot configuration. With the
default Windows Visual Studio build, launch it from the repository root:

```powershell
$env:PATH = "C:\Qt\6.8.3\msvc2022_64\bin;$env:PATH"
.\build\windows-vs2022\bin\Release\wpiemu_editor_demo.exe --select wheel:fl
```

If your Qt kit is installed elsewhere, use its `bin` directory instead.
In the inspector, open the motor choice and confirm it contains **NEO (practice copy)**.
Do not select it in a real robot configuration. This visual inspection checks that the
choice is displayed; it does not validate a persisted project, Quick 3D rendering, or
robot integration. Close the demo when finished.

## Run checks and clean up safely

Run the repository checks relevant to your change:

```powershell
python tools\check_boundaries.py
python tools\check_docs.py
python tools\check_versions.py
git diff --check
```

Expected result: the checks report success and `git diff --check` prints no whitespace
errors. Boundary checks enforce dependency rules; documentation checks validate local
links and anchors; version checks preserve the repository's pins. None of these checks
prove the motor's physical accuracy.

Now remove only the temporary row, using your editor. Review the diff and confirm no
unrelated changes were removed. Rebuild and rerun the focused test to verify the
practice row is gone:

```powershell
cmake --build --preset windows-vs2022-release --parallel
ctest --preset windows-vs2022-release -R "^every catalog motor reproduces its stall and free operating points$" --output-on-failure
```

Do not use `git reset`, `git clean`, or a blanket restore command to clean up. If you
want to make a real preset change, first replace the practice values with sourced motor
data and independent test expectations, then review and stage only your intended files.

Next: [choose an extension seam](../extending.md).
