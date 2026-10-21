"""Shared setup for the tool tests.

Importing this module first puts tools/ on sys.path, so the tools and `common` import like they do when a
script runs, and `unittest discover` needs no extra configuration.
"""
import importlib.util
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import common  # noqa: E402

REPO = common.REPO_ROOT


def load(name, relative):
    """Import a script that is not on sys.path, such as one under packaging/."""
    spec = importlib.util.spec_from_file_location(name, REPO / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def scratch_directory(case, prefix):
    """A temporary directory under the git-ignored build/ folder, removed when the test finishes."""
    (REPO / "build").mkdir(exist_ok=True)
    directory = tempfile.TemporaryDirectory(prefix=prefix, dir=REPO / "build")
    case.addCleanup(directory.cleanup)
    return Path(directory.name).resolve()
