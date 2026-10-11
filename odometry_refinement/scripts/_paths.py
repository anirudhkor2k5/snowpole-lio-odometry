"""Shared paths for the experiment scripts."""

from __future__ import annotations

import sys
from pathlib import Path


def framework_root() -> Path:
    return Path(__file__).resolve().parents[1]


def repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


def install_src() -> None:
    src = framework_root() / "src"
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))
