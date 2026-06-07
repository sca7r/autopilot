"""Locators for files bundled with the autopilot package.

Use these constants and helpers instead of hard-coding paths so the
package works correctly whether it is installed from a wheel or run
in an editable development checkout.
"""

from __future__ import annotations

import pathlib

# src/autopilot/
_PACKAGE_DIR = pathlib.Path(__file__).resolve().parent

CONFIG_DIR: pathlib.Path = _PACKAGE_DIR / "config"
MODELS_DIR: pathlib.Path = _PACKAGE_DIR / "models"

MPC_CONFIG: pathlib.Path = CONFIG_DIR / "mpc.yaml"
MUSHR_MODEL: pathlib.Path = MODELS_DIR / "mushr" / "mush_nano.xml"


def config_path(name: str) -> pathlib.Path:
    """Return the absolute path to a bundled config file by name.

    Args:
        name: File name within the ``config`` directory, e.g. ``"mpc.yaml"``.

    Returns:
        Absolute path to the requested config file.

    Raises:
        FileNotFoundError: If the file does not exist.
    """
    path = CONFIG_DIR / name
    if not path.exists():
        raise FileNotFoundError(f"Bundled config not found: {path}")
    return path
