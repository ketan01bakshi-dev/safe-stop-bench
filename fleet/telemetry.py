"""Fleet telemetry (P4): safety events as JSON, to a list always and to an MQTT broker when there is one.

Events: boot (with the firmware version and whether the safe state was restored), staged / commit / rollback of an update, resets,
and SAF_Status changes (state, cause) read from the bus frames that board A mirrors. Topic: `ssb/fleet/<device>/<event>`.
Publishing is best effort and never blocks or fails a test: with no paho-mqtt installed, or no broker, events are only kept in memory.
"""
from __future__ import annotations

import json
import time

from ssb import e2e
from ssb.canio import CAUSES, STATES


class Telemetry:
    def __init__(self, broker: str | None = None, port: int = 1883, client_id: str = "ssb-fleet"):
        self.events: list[dict] = []
        self.published = 0
        self._client = None
        if broker:
            try:
                import paho.mqtt.client as mqtt
                try:
                    self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id)
                except AttributeError:   # paho 1.x
                    self._client = mqtt.Client(client_id)  # type: ignore[arg-type]
                self._client.connect(broker, port, 30)
                self._client.loop_start()
            except Exception:   # noqa: BLE001  no paho, no broker: events stay in memory
                self._client = None

    def emit(self, device: str, event: str, **fields) -> dict:
        e = {"ts": round(time.time(), 3), "device": device, "event": event, **fields}
        self.events.append(e)
        if self._client is not None:
            try:
                self._client.publish(f"ssb/fleet/{device}/{event}", json.dumps(e), qos=1)
                self.published += 1
            except Exception:   # noqa: BLE001
                pass
        return e

    def absorb(self, events: list[dict]) -> None:
        """Take over a device's own event list (EmuBoard.events)."""
        for e in events:
            self.emit(e["device"], e["event"], **{k: v for k, v in e.items() if k not in ("device", "event")})

    def close(self) -> None:
        if self._client is not None:
            self._client.loop_stop()
            self._client.disconnect()



def status_events(device: str, frames: list[tuple[int, int, bytes]]) -> list[dict]:
    """SAF_Status (0x201) frames (t_ms, can id, data) -> one event per change of state or cause, plus how many frames failed their check."""
    out, rx, last, rejected = [], e2e.StatusReceiver(), None, 0
    for t, cid, data in frames:
        if cid != 0x201:
            continue
        if not rx.check(data):
            rejected += 1
            continue
        s = e2e.status_unpack(data)
        key = (s.get("state"), s.get("cause"))
        if key != last:
            out.append({"device": device, "event": "saf_state", "t_ms": t, "state": STATES[s["state"]], "cause": CAUSES[s["cause"]] if s["cause"] < len(CAUSES) else s["cause"]})
            last = key
    if rejected:
        out.append({"device": device, "event": "status_e2e_rejects", "count": rejected})
    return out
