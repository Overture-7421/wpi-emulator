#!/usr/bin/env python3
"""Architecture boundary check for wpi-emulator.

Rules (see docs/decisions/0004-modular-boundaries.md and 0005-sdk-freeze.md):
  * core/, app/, sdk/ (and every non-plugin module) never include anything from plugins/.
  * plugins/<name>/ may include only wpiemu/sdk/** and its own files; never core, app,
    ipc, bridges, halsim_ext, or another plugin.
  * The same applies to CMake: plugins link only wpiemu_sdk; nothing else links wpiemu_plugin_*.
  * Aggregation seam: files directly in plugins/ (target wpiemu_plugins_builtin) may see every
    plugin; app may link that target; nobody else may reference it.
  * Non-app headers must not use identifiers that Qt turns into macros (signals, slots, emit, foreach).

Owners are directories; ALLOWED maps an owner to the owners it may depend on.
Run:  check_boundaries.py [--root DIR]     |     check_boundaries.py --self-test
"""
from __future__ import annotations

import argparse
import os
import posixpath
import re
import sys
from pathlib import Path

SOURCE_EXTS = {".h", ".hh", ".hpp", ".hxx", ".inl", ".c", ".cc", ".cpp", ".cxx", ".mm", ".m"}
CMAKE_NAMES = {"CMakeLists.txt"}
CMAKE_EXTS = {".cmake"}

# Top-level directories that are not scanned.
IGNORED_TOP = {"docs", "spikes", "packaging", "tools", "tests", "cmake", "build", "out", "node_modules"}
IGNORED_PARTS = {"_deps", ".fetchcontent-cache", ".git", ".github", ".vs", ".idea"}

# owner -> owners it may reference (itself is always allowed).
ALLOWED: dict[str, set[str]] = {
    "sdk": set(),
    "core": {"sdk"},
    "ipc": {"sdk"},
    "halsim_ext": {"sdk", "ipc"},
    "bridges/hal": {"sdk"},
    "bridges/vendor": {"sdk"},
    "app": {"sdk", "core", "ipc", "plugins"},
}
# Any plugin ("plugins/<name>") may reference only these (plus itself).
PLUGIN_ALLOWED = {"sdk"}
# The plugins/ aggregation seam (files directly in plugins/, target wpiemu_plugins_builtin) is the one
# place that may see every plugin (ADR 0005). app may link it (never include it); nobody else may
# reference it, and plugins may not reference it.
AGGREGATOR = "plugins"
AGGREGATOR_ALLOWED = {"sdk", AGGREGATOR, "plugins/*"}

MODULE_OWNERS = ["sdk", "core", "ipc", "halsim_ext", "bridges/hal", "bridges/vendor", "app"]

INCLUDE_RE = re.compile(r'^\s*#\s*include\s*([<"])([^>"]+)[>"]', re.MULTILINE)
QT_KEYWORD_RE = re.compile(r"\b(signals|slots|emit|foreach|Q_SIGNALS|Q_SLOTS|Q_EMIT|Q_FOREACH)\b")
# Namespaced public header convention: wpiemu/<module>/..., wpiemu/bridges/<hal|vendor>/...,
# wpiemu/plugins/<name>/...
NAMESPACED_RE = re.compile(r"^wpiemu/(?:(plugins)/([^/]+)|(bridges)/([^/]+)|([^/]+))/")

CMAKE_LINK_RE = re.compile(
    r"target_link_libraries\s*\(\s*([^\s)]+)([^)]*)\)", re.IGNORECASE | re.DOTALL
)
CMAKE_INCLUDE_DIRS_RE = re.compile(r"target_include_directories\s*\(([^)]*)\)", re.IGNORECASE | re.DOTALL)
WPIEMU_TARGET_RE = re.compile(r"\bwpiemu::([A-Za-z0-9_]+)|\b(wpiemu_[A-Za-z0-9_]+|halsim_wpiemu)\b")

# CMake target -> owner (ownership is by naming convention).
TARGET_OWNERS = {
    "wpiemu_sdk": "sdk",
    "wpiemu_core": "core",
    "wpiemu_ipc": "ipc",
    "halsim_wpiemu": "halsim_ext",
    "wpiemu_bridges_hal": "bridges/hal",
    "wpiemu_bridges_vendor": "bridges/vendor",
    "wpiemu_app": "app",
    "wpiemu_plugins_builtin": AGGREGATOR,
}


def owner_of_path(rel: str) -> str | None:
    """Owner for a repo-relative posix path, or None if not part of a checked module."""
    parts = rel.split("/")
    if parts[0] == "plugins" and len(parts) >= 3:
        return f"plugins/{parts[1]}"
    if parts[0] == "plugins" and len(parts) == 2:
        return AGGREGATOR
    if parts[0] == "bridges" and len(parts) >= 3:
        return f"bridges/{parts[1]}"
    if parts[0] in ("sdk", "core", "ipc", "halsim_ext", "app"):
        return parts[0]
    return None


def owner_of_target(name: str) -> str | None:
    if name in TARGET_OWNERS:
        return TARGET_OWNERS[name]
    m = re.fullmatch(r"wpiemu_plugin_(.+)", name)
    if m:
        plugin = m.group(1)
        # plugins/_template builds the target wpiemu_plugin_template.
        return f"plugins/{'_template' if plugin == 'template' else plugin}"
    return None


def is_plugin(owner: str | None) -> bool:
    return owner is not None and owner.startswith("plugins/")


def permits(allowed: set[str], dep: str) -> bool:
    return dep in allowed or ("plugins/*" in allowed and dep.startswith("plugins/"))


def allowed_deps(owner: str) -> set[str]:
    if owner == AGGREGATOR:
        return set(AGGREGATOR_ALLOWED)
    if is_plugin(owner):
        return PLUGIN_ALLOWED | {owner}
    return ALLOWED.get(owner, set()) | {owner}


def _plugin_dir_names(known: set[str]) -> set[str]:
    return {p.split("/")[1] for p in known if p.startswith("plugins/") and p.count("/") >= 2}


def classify_include(src_rel: str, target: str, quoted: bool, known: set[str]) -> str | None:
    """Return the owner an #include refers to, or None for system/third-party headers."""
    src_dir = posixpath.dirname(src_rel)
    t = target.replace("\\", "/")

    if quoted:
        # Relative to the including file.
        cand = posixpath.normpath(posixpath.join(src_dir, t))
        if cand in known:
            return owner_of_path(cand)
        # Relative to repo root (e.g. "plugins/game_2026/foo.h").
        if posixpath.normpath(t) in known:
            return owner_of_path(posixpath.normpath(t))
        # Relative to any module's include/ or src/ root.
        for k in known:
            for marker in ("/include/", "/src/"):
                idx = k.find(marker)
                if idx >= 0 and k[idx + len(marker):] == posixpath.normpath(t):
                    return owner_of_path(k)

    m = NAMESPACED_RE.match(t)
    if m:
        if m.group(1):
            return f"plugins/{m.group(2)}"
        if m.group(3):
            return f"bridges/{m.group(4)}"
        return m.group(5)

    # Textual fallbacks for path-style includes ("../plugins/x/y.h", "plugins/x/y.h").
    norm = posixpath.normpath(posixpath.join(src_dir, t)) if quoted else posixpath.normpath(t)
    for candidate in (norm, posixpath.normpath(t)):
        parts = candidate.split("/")
        if len(parts) >= 3 and parts[0] == "plugins":
            return f"plugins/{parts[1]}"
        if len(parts) >= 3 and parts[0] == "bridges":
            return f"bridges/{parts[1]}"
        if len(parts) >= 2 and quoted and parts[0] in ("core", "app", "ipc", "sdk", "halsim_ext"):
            return parts[0]
    return None


def scan_source(rel: str, text: str, known: set[str]) -> list[str]:
    owner = owner_of_path(rel)
    if owner is None:
        return []
    allowed = allowed_deps(owner)
    errors = []
    for m in INCLUDE_RE.finditer(text):
        quoted = m.group(1) == '"'
        target = m.group(2)
        dep = classify_include(rel, target, quoted, known)
        if dep is None or (permits(allowed, dep) and not (dep == AGGREGATOR and owner != AGGREGATOR)):
            continue
        line = text.count("\n", 0, m.start()) + 1
        errors.append(
            f"{rel}:{line}: '{owner}' must not include '{target}' (belongs to '{dep}'); "
            f"allowed: {sorted(allowed)}"
        )
    return errors


def scan_cmake(rel: str, text: str) -> list[str]:
    owner = owner_of_path(rel)
    if owner is None:
        return []
    allowed = allowed_deps(owner)
    # Drop comments so documentation does not trigger the check.
    text = re.sub(r"#[^\n]*", "", text)
    errors = []
    for m in CMAKE_LINK_RE.finditer(text):
        line = text.count("\n", 0, m.start()) + 1
        for alias, name in WPIEMU_TARGET_RE.findall(m.group(2)):
            name = f"wpiemu_{alias}" if alias else name
            dep = owner_of_target(name)
            if dep is None or permits(allowed, dep):
                continue
            errors.append(
                f"{rel}:{line}: '{owner}' must not link '{name}' (belongs to '{dep}'); "
                f"allowed: {sorted(allowed)}"
            )
    for m in CMAKE_INCLUDE_DIRS_RE.finditer(text):
        line = text.count("\n", 0, m.start()) + 1
        for token in re.findall(r"[^\s()]+", m.group(1)):
            norm = token.replace("\\", "/")
            if re.search(r"(^|/)\.\./", norm) or re.search(r"\$\{(CMAKE|PROJECT)_SOURCE_DIR\}", norm):
                errors.append(
                    f"{rel}:{line}: '{owner}' must not add include directory '{token}' "
                    f"that reaches outside its own folder"
                )
    return errors


def scan_qt_keywords(rel: str, text: str) -> list[str]:
    """Headers shared with the Qt app must not use identifiers Qt turns into macros
    (`signals`, `slots`, `emit`, `foreach`, ...): they break when included after a Qt header."""
    owner = owner_of_path(rel)
    if owner is None or owner == "app" or not rel.endswith((".h", ".hh", ".hpp", ".hxx", ".inl")):
        return []
    stripped = re.sub(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\\n])*"', lambda m: re.sub(r"[^\n]", " ", m.group(0)), text, flags=re.DOTALL)
    errors = []
    for m in QT_KEYWORD_RE.finditer(stripped):
        line = stripped.count("\n", 0, m.start()) + 1
        errors.append(f"{rel}:{line}: identifier '{m.group(0)}' collides with a Qt keyword macro; rename it")
    return errors


def collect_files(root: Path) -> list[str]:
    out = []
    for dirpath, dirnames, filenames in os.walk(root):
        rel_dir = Path(dirpath).relative_to(root).as_posix()
        dirnames[:] = [
            d for d in dirnames
            if d not in IGNORED_PARTS and not (rel_dir == "." and (d in IGNORED_TOP or d.startswith("build")))
        ]
        for f in filenames:
            rel = f if rel_dir == "." else f"{rel_dir}/{f}"
            out.append(rel)
    return out


def check_tree(root: Path) -> list[str]:
    files = collect_files(root)
    known = set(files)
    errors: list[str] = []
    for rel in files:
        name = posixpath.basename(rel)
        ext = posixpath.splitext(rel)[1].lower()
        path = root / rel
        if owner_of_path(rel) is None:
            continue
        if ext in SOURCE_EXTS:
            text = path.read_text(encoding="utf-8", errors="replace")
            errors += scan_source(rel, text, known)
            errors += scan_qt_keywords(rel, text)
        elif name in CMAKE_NAMES or ext in CMAKE_EXTS:
            errors += scan_cmake(rel, path.read_text(encoding="utf-8", errors="replace"))
    return errors


def self_test() -> int:
    known = {
        "sdk/include/wpiemu/sdk/a.h",
        "core/include/wpiemu/core/c.h",
        "core/src/c.cpp",
        "plugins/game_2026/include/wpiemu/plugins/game_2026/p.h",
        "plugins/game_2026/src/p.cpp",
        "plugins/game_2027/include/wpiemu/plugins/game_2027/q.h",
        "plugins/builtin_plugins.cpp",
    }
    cases = [
        # (rel, text, expected_error_count)
        ("core/src/c.cpp", '#include "wpiemu/core/c.h"\n#include <wpiemu/sdk/a.h>\n#include <vector>\n', 0),
        ("core/src/c.cpp", '#include "wpiemu/plugins/game_2026/p.h"\n', 1),
        ("core/src/c.cpp", '#include "../../plugins/game_2026/src/p.cpp"\n', 1),
        ("app/src/main.cpp", '#include <wpiemu/plugins/game_2026/p.h>\n', 1),
        ("sdk/include/wpiemu/sdk/a.h", '#include "wpiemu/core/c.h"\n', 1),
        ("plugins/game_2026/src/p.cpp", '#include "wpiemu/sdk/a.h"\n#include "wpiemu/plugins/game_2026/p.h"\n', 0),
        ("plugins/game_2026/src/p.cpp", '#include "wpiemu/core/c.h"\n', 1),
        ("plugins/game_2026/src/p.cpp", '#include <wpiemu/app/x.h>\n', 1),
        ("plugins/game_2026/src/p.cpp", '#include "wpiemu/plugins/game_2027/q.h"\n', 1),
        ("plugins/game_2026/src/p.cpp", '#include "../../game_2027/include/wpiemu/plugins/game_2027/q.h"\n', 1),
        ("plugins/game_2026/src/p.cpp", '#include "wpiemu/bridges/hal/x.h"\n', 1),
        ("plugins/game_2026/src/p.cpp", '#include <box3d/box3d.h>\n#include <string>\n', 0),
        # aggregation seam: may see sdk + every plugin; nobody else may include it
        ("plugins/builtin_plugins.cpp", '#include "wpiemu/sdk/game/builtin_plugins.h"\n#include "wpiemu/plugins/game_2026/p.h"\n#include "wpiemu/plugins/game_2027/q.h"\n', 0),
        ("plugins/builtin_plugins.cpp", '#include "wpiemu/core/c.h"\n', 1),
        ("app/src/main.cpp", '#include "wpiemu/plugins/builtin_plugins.h"\n', 1),
        ("plugins/game_2026/src/p.cpp", '#include "wpiemu/plugins/builtin_plugins.h"\n', 1),
    ]
    failures = 0
    for rel, text, expected in cases:
        got = len(scan_source(rel, text, known))
        if got != expected:
            failures += 1
            print(f"SELF-TEST FAIL source {rel}: expected {expected} errors, got {got}\n{text}")

    cmake_cases = [
        ("plugins/game_2026/CMakeLists.txt",
         "target_link_libraries(wpiemu_plugin_game_2026 PUBLIC wpiemu_sdk)\n", 0),
        ("plugins/game_2026/CMakeLists.txt",
         "target_link_libraries(wpiemu_plugin_game_2026 PUBLIC wpiemu_sdk wpiemu_core)\n", 1),
        ("plugins/game_2026/CMakeLists.txt",
         "target_link_libraries(wpiemu_plugin_game_2026 PRIVATE wpiemu::app)\n", 1),
        ("plugins/game_2026/CMakeLists.txt",
         "target_link_libraries(wpiemu_plugin_game_2026 PRIVATE wpiemu_plugin_game_2027)\n", 1),
        ("plugins/game_2026/CMakeLists.txt",
         "# target_link_libraries(x PUBLIC wpiemu_core)\n", 0),
        ("core/CMakeLists.txt",
         "target_link_libraries(wpiemu_core PUBLIC wpiemu_sdk PRIVATE box3d)\n", 0),
        ("core/CMakeLists.txt",
         "target_link_libraries(wpiemu_core PUBLIC wpiemu_plugin_game_2026)\n", 1),
        ("app/CMakeLists.txt",
         "target_link_libraries(wpiemu_app PRIVATE wpiemu_core wpiemu_ipc wpiemu_sdk Qt6::Core)\n", 0),
        ("app/CMakeLists.txt",
         "target_link_libraries(wpiemu_app PRIVATE wpiemu::plugin_game_2026)\n", 1),
        ("plugins/_template/CMakeLists.txt",
         "target_link_libraries(wpiemu_plugin_template PUBLIC wpiemu::sdk)\n", 0),
        ("app/CMakeLists.txt",
         "target_link_libraries(wpiemu_app PRIVATE wpiemu_plugin_game_2026)\n", 1),
        ("sdk/CMakeLists.txt",
         "target_link_libraries(wpiemu_sdk INTERFACE wpiemu_core)\n", 1),
        ("plugins/game_2026/CMakeLists.txt",
         "target_include_directories(wpiemu_plugin_game_2026 PUBLIC ../../core/include)\n", 1),
        # aggregation seam
        ("plugins/CMakeLists.txt",
         "target_link_libraries(wpiemu_plugins_builtin PUBLIC wpiemu_sdk PRIVATE wpiemu_plugin_game_2026 wpiemu_plugin_game_2027)\n", 0),
        ("plugins/CMakeLists.txt",
         "target_link_libraries(wpiemu_plugins_builtin PRIVATE wpiemu_core)\n", 1),
        ("app/CMakeLists.txt",
         "target_link_libraries(wpiemu_app PRIVATE wpiemu_core wpiemu_ipc wpiemu_sdk wpiemu_plugins_builtin)\n", 0),
        ("core/CMakeLists.txt",
         "target_link_libraries(wpiemu_core PUBLIC wpiemu_plugins_builtin)\n", 1),
        ("plugins/game_2026/CMakeLists.txt",
         "target_link_libraries(wpiemu_plugin_game_2026 PUBLIC wpiemu::plugins_builtin)\n", 1),
        ("sdk/CMakeLists.txt",
         "target_link_libraries(wpiemu_sdk INTERFACE wpiemu_plugins_builtin)\n", 1),
    ]
    for rel, text, expected in cmake_cases:
        got = len(scan_cmake(rel, text))
        if got != expected:
            failures += 1
            print(f"SELF-TEST FAIL cmake {rel}: expected {expected} errors, got {got}\n{text}")

    kw_cases = [
        ("sdk/include/wpiemu/sdk/a.h", "std::vector<int> signals;\n", 1),
        ("sdk/include/wpiemu/sdk/a.h", "int slots() const;\n", 1),
        ("sdk/include/wpiemu/sdk/a.h", "// the slots and signals\nconst char* s = \"slots\";\n/* emit */ int ok;\n", 0),
        ("plugins/game_2026/include/wpiemu/plugins/game_2026/p.h", "int emit;\n", 1),
        ("app/src/main.cpp", "int signals;\n", 0),
        ("core/src/c.cpp", "int signals;\n", 0),
    ]
    for rel, text, expected in kw_cases:
        got = len(scan_qt_keywords(rel, text))
        if got != expected:
            failures += 1
            print(f"SELF-TEST FAIL qt-keyword {rel}: expected {expected} errors, got {got}\n{text}")

    print("self-test:", "FAILED" if failures else "ok")
    return 1 if failures else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=Path(__file__).resolve().parent.parent)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    errors = check_tree(args.root)
    if errors:
        print("Architecture boundary violations:")
        for e in errors:
            print("  " + e)
        return 1
    print("boundaries: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
