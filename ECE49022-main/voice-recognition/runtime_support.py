"""Shared import and credential-loading helpers for voice scripts.

This module is relevant because several voice files need repo-root resources
such as `keys.py`, and this keeps that path and import setup consistent.
"""

from pathlib import Path
import importlib
import sys


VOICE_DIR = Path(__file__).resolve().parent
REPO_ROOT = VOICE_DIR.parent


def ensure_repo_root_on_path():
    """Allow voice scripts to import repo-root modules such as keys.py."""
    repo_root = str(REPO_ROOT)
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)


def load_keys_module():
    """Load the optional repo-local keys.py module if it exists."""
    ensure_repo_root_on_path()
    try:
        return importlib.import_module("keys")
    except ModuleNotFoundError:
        return None
