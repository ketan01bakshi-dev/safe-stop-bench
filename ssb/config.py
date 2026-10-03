"""Load a JSON config; a file may inherit another with "_inherits" and override parts of it."""
from __future__ import annotations

import copy
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def _merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def load(path: str | Path = "config/default.json") -> dict:
    path = Path(path)
    if not path.is_absolute():
        path = ROOT / path
    data = json.loads(path.read_text(encoding="utf-8"))
    if "_inherits" in data:
        data = _merge(load(path.parent / data["_inherits"]), {k: v for k, v in data.items() if k != "_inherits"})
    return data
