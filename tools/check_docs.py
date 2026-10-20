#!/usr/bin/env python3
"""Check local documentation links, repository paths, and build identifiers."""
from __future__ import annotations

import argparse
import html
import re
import subprocess
import sys
from pathlib import Path
from urllib.parse import unquote, urlsplit


TEXT_SUFFIXES = {".md", ".h", ".hpp", ".cpp", ".cc", ".qml", ".cmake",
                 ".py", ".ps1", ".sh", ".yml", ".yaml", ".gradle", ".txt"}
GENERATED_ROOTS = {"build", "out", "spikes", ".wpiemu", ".gradle", "bin"}
INLINE = re.compile(r"(`+)(.+?)\1")
LINK = re.compile(r"!?\[[^\]\n]*\]\(\s*(<[^>]+>|[^\s)]+)")
REFERENCE = re.compile(r"^\s{0,3}\[([^\]]+)\]:\s*(<[^>]+>|[^\s]+)")
REFERENCE_LINK = re.compile(r"!?\[([^\]\n]+)\]\[([^\]\n]*)\]")
ADR = re.compile(r"\bADR[ -]+\d{4}\b|decisions[/\\]\d{4}-", re.IGNORECASE)
TARGET = re.compile(r"\b(?:wpiemu_[A-Za-z0-9_]+|halsim_wpiemu)\b")
VARIABLE = re.compile(r"\bWPIEMU_[A-Z0-9_]+\b")
COMMENT_OR_STRING = (
    r'R"(?P<delimiter>[^ ()\\\t\r\n]{0,16})\([\s\S]*?\)(?P=delimiter)"'
    r'|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\''
    r'|//[^\n]*|/\*[\s\S]*?\*/')
C_TOKENS = re.compile(COMMENT_OR_STRING)
SCRIPT_TOKENS = re.compile(COMMENT_OR_STRING + r"|#[^\n]*")


def comments(path: Path, text: str):
    tokens = C_TOKENS if path.suffix in {".h", ".hpp", ".cpp", ".cc", ".qml", ".gradle"} else SCRIPT_TOKENS
    for match in tokens.finditer(text):
        if match[0].startswith(("//", "/*", "#")):
            yield match


def prose(text: str) -> list[tuple[int, str]]:
    lines = []
    fence = ""
    for number, line in enumerate(text.splitlines(), 1):
        marker = re.match(r"^\s{0,3}(`{3,}|~{3,})(.*)$", line)
        if marker:
            ticks, tail = marker.groups()
            if not fence:
                fence = ticks
            elif ticks[0] == fence[0] and len(ticks) >= len(fence) and not tail.strip():
                fence = ""
            lines.append((number, ""))
        else:
            lines.append((number, "" if fence else line))
    return lines


def anchors(text: str) -> set[str]:
    found = set()
    for _, line in prose(text):
        heading = re.match(r"^\s{0,3}#{1,6}\s+(.+?)(?:\s+#+)?\s*$", line)
        if heading:
            title = re.sub(r"<[^>]*>", "", html.unescape(heading[1])).lower()
            title = re.sub(r"!?\[([^\]]*)\]\([^)]*\)", r"\1", title)
            base = re.sub(r"[^\w -]", "", title).replace(" ", "-")
            name, suffix = base, 0
            while name in found:
                suffix += 1
                name = f"{base}-{suffix}"
            found.add(name)
        found.update(re.findall(r'<a\s+(?:id|name)=["\']([^"\']+)["\']', line))
    return found


def check(root: Path, files: list[Path]) -> list[str]:
    root = root.resolve()
    files = [path for path in files if path.is_file()]
    texts = {path: path.read_text(encoding="utf-8") for path in files
             if path.suffix in TEXT_SUFFIXES or path.name == "CMakeLists.txt"}
    documents = {path: text for path, text in texts.items() if path.suffix == ".md"}
    if not documents:
        return ["docs: no Markdown files found; use the repository root"]
    headings = {path: anchors(text) for path, text in documents.items()}
    cmake = "\n".join(re.sub(r"#[^\n]*", "", text) for path, text in texts.items()
                      if path.suffix == ".cmake" or path.name == "CMakeLists.txt")
    targets = set(re.findall(r"(?:qt_)?add_(?:library|executable)\s*\(\s*([\w:]+)",
                             cmake, re.IGNORECASE))
    source = "\n".join(text for path, text in texts.items()
                       if path.suffix != ".md" and path.name not in
                       {"check_docs.py", "test_docs.py"})
    variables = set(VARIABLE.findall(source))
    roots = {path.relative_to(root).parts[0] for path in files
             if len(path.relative_to(root).parts) > 1} - GENERATED_ROOTS
    errors = []

    def report(path: Path, number: int, message: str) -> None:
        errors.append(f"{path.relative_to(root).as_posix()}:{number}: {message}")

    def link(path: Path, number: int, destination: str) -> None:
        parsed = urlsplit(destination.strip("<>"))
        if parsed.scheme or parsed.netloc:
            return
        value = unquote(parsed.path)
        target = ((root / value.lstrip("/")) if value.startswith("/")
                  else path.parent / value).resolve() if value else path
        if not target.is_relative_to(root):
            report(path, number, f"link leaves repository: {destination}")
        elif not target.exists():
            report(path, number, f"missing link target: {destination}")
        elif parsed.fragment:
            if target.is_dir():
                target /= "README.md"
            if target.suffix == ".md" and unquote(parsed.fragment) not in headings.get(target, set()):
                report(path, number, f"missing heading: {destination}")

    for path, text in texts.items():
        if not path.is_relative_to(root / "docs" / "decisions"):
            if path in documents:
                citations = enumerate(text.splitlines(), 1)
            else:
                citations = ((text.count("\n", 0, match.start()) + 1, match[0])
                             for match in comments(path, text))
            for number, comment in citations:
                if ADR.search(comment):
                    report(path, number, "use a current reference page, not a numbered ADR citation")
                if path not in documents:
                    for reference in re.findall(r"\bdocs[/\\][\w./\\-]+\.md", comment):
                        if not (root / reference.replace("\\", "/")).is_file():
                            report(path, number, f"missing documentation path: {reference}")
        if path not in documents:
            continue
        lines = prose(text)
        references = {}
        for number, line in lines:
            definition = REFERENCE.match(line)
            if definition:
                references[" ".join(definition[1].lower().split())] = definition[2]
                link(path, number, definition[2])
        for number, line in lines:
            for match in INLINE.finditer(line):
                value = match[2]
                if not re.search(r"[\s<>*${}]", value):
                    for name in TARGET.findall(value):
                        if name not in targets:
                            report(path, number, f"unknown CMake target: {name}")
                    for name in VARIABLE.findall(value):
                        if name not in variables:
                            report(path, number, f"unknown build/environment variable: {name}")
                    normalized = value.replace("\\", "/").rstrip("/")
                    if normalized.split("/")[0] in roots and "/" in normalized:
                        if not (root / normalized).exists():
                            report(path, number, f"missing repository path: {value}")
            plain = INLINE.sub("", line)
            for match in LINK.finditer(plain):
                link(path, number, match[1])
            for match in REFERENCE_LINK.finditer(plain):
                label = " ".join((match[2] or match[1]).lower().split())
                if label not in references:
                    report(path, number, f"undefined link reference: {label}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    root = args.root.resolve()
    try:
        names = subprocess.check_output(
            ["git", "-C", str(root), "ls-files", "-z", "--cached", "--others", "--exclude-standard"],
            stderr=subprocess.PIPE).decode("utf-8").split("\0")
        errors = check(root, [root / name for name in dict.fromkeys(names) if name])
    except (OSError, UnicodeError, subprocess.CalledProcessError) as error:
        print(f"docs: cannot check repository: {error}", file=sys.stderr)
        return 2
    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    print("docs: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
