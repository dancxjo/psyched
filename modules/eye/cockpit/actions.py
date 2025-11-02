"""Custom cockpit actions for the eye module."""

from __future__ import annotations

import os
import stat
from pathlib import Path
from typing import Any, Dict, List

from cockpit.actions import ActionContext, ActionResult


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeDecodeError):
        return ""


def _device_metadata(entry: Path, sys_root: Path) -> Dict[str, Any]:
    name = entry.name
    sys_entry = sys_root / entry.name
    label = _read_text(sys_entry / "name") or name
    device_path = str(entry)

    readable = os.access(device_path, os.R_OK)
    writable = os.access(device_path, os.W_OK)

    capabilities = _read_text(sys_entry / "uevent")
    subtype = _read_text(sys_entry / "device" / "uevent")

    return {
        "id": name,
        "path": device_path,
        "label": label,
        "readable": bool(readable),
        "writable": bool(writable),
        "uevent": capabilities,
        "device_uevent": subtype,
    }


async def list_video_devices_action(*, _context: ActionContext, **_: Any) -> ActionResult:
    """Return metadata describing available V4L2 video devices."""

    dev_root = Path("/dev")
    sys_root = Path("/sys/class/video4linux")
    devices: List[Dict[str, Any]] = []

    try:
        entries = sorted(dev_root.glob("video*"))
    except OSError:
        entries = []

    for entry in entries:
        try:
            st = entry.stat()
        except OSError:
            continue
        if not stat.S_ISCHR(st.st_mode):
            continue
        devices.append(_device_metadata(entry, sys_root))

    return ActionResult(payload={"devices": devices})


__all__ = ["list_video_devices_action"]
