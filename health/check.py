"""Bench health (P2): is a failing verdict about the product, or about the bench?

A run is a SUSPECT BENCH run when
- a board reports firmware other than the inventory says (the real precedent: the ESP32 was not reflashed after v2.6),
- a board that the inventory lists reported nothing at all in a HiL run, or
- a scenario needed bench-fault reruns (host stall, lost frames): that scenario's verdict is suspect, the others are not.

Only the HiL level has boards; every other level has nothing to compare and is reported healthy.
"""
from __future__ import annotations

import json
import re
from pathlib import Path

INVENTORY = Path(__file__).with_name("inventory.json")


def load_inventory(path: Path = INVENTORY) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _version(text: str | None) -> str | None:
    m = re.match(r"\s*([A-Za-z]+\s+\d+(?:\.\d+)*)", text or "")
    return m.group(1) if m else None


def firmware_problems(firmware: dict | None, level: str, inventory: dict | None = None) -> list[str]:
    """Reasons the boards are not what the inventory expects (empty = healthy or not a HiL run)."""
    if level != "hil":
        return []
    inv = inventory or load_inventory()
    out = []
    for board, want in ((k, v) for k, v in inv.items() if not k.startswith("_")):
        got = (firmware or {}).get(board)
        if not got or got == "?":
            out.append(f"{board}: no firmware hello (inventory expects {want['firmware']})")
        elif _version(got) != want["firmware"]:
            out.append(f"{board}: {_version(got) or got} is not the inventory's {want['firmware']}")
    return out


def bench_health(manifest: dict, results: list[dict], inventory: dict | None = None) -> dict:
    """{'fleet': [reasons that make the whole run suspect], 'by_key': {scenario key: [bench faults that forced reruns]}}."""
    level = (manifest.get("run") or {}).get("level") or manifest.get("level") or ""
    return {"fleet": firmware_problems(manifest.get("firmware"), level, inventory),
            "by_key": {r["key"]: list(r["bench_faults"]) for r in results if r.get("bench_faults")}}
