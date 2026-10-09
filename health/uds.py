"""UDS version read from the two boards (P2). The PC sends standard CAN frames through board A's tester port ('T'), board B answers on
the real bus (0x7E2 -> 0x7EA) and board A answers for itself (0x7E3 -> 0x7EB); every answer comes back to the PC as an 'M' frame
that board A forwards. ISO-TP single frames only (hil/SafeStopCore/src/ssc_uds.h).

Asked of each board: TesterPresent (0x3E 00), ReadDataByIdentifier F195 (software version) and F18C (serial). The version is
compared with the inventory AND with the USB hello: a firmware whose two answers disagree is not trusted.
"""
from __future__ import annotations

import re
import struct
import time

from ssb.hil import Parser, frame_msg

NODES = {"board_b": {"req": 0x7E2, "resp": 0x7EA}, "board_a": {"req": 0x7E3, "resp": 0x7EB}}
FUNCTIONAL = 0x7DF
NRC = {0x11: "service not supported", 0x12: "sub-function not supported", 0x13: "incorrect length", 0x31: "request out of range"}


class Tester:
    """Talks UDS through board A's serial link. `link` is an ssb.hil SerialLink (or anything with write/read)."""

    def __init__(self, link, timeout_s: float = 0.4):
        self.link, self.timeout_s, self.parser = link, timeout_s, Parser()

    def _send(self, can_id: int, data: bytes) -> None:
        self.link.write(frame_msg("T", struct.pack("<HB", can_id, len(data)) + data), 0)

    def request(self, can_id: int, payload: bytes, resp_ids: set[int]) -> dict[int, bytes]:
        """Send one single-frame request (ISO-TP PCI + payload); return {response id: UDS payload} seen within the timeout."""
        self.parser.feed(self.link.read())   # drop what is already there (hello, earlier frames)
        self._send(can_id, bytes([len(payload)]) + payload)
        seen: dict[int, bytes] = {}
        end = time.time() + self.timeout_s
        while time.time() < end and set(seen) != resp_ids:
            for kind, p in self.parser.feed(self.link.read()):
                if kind == "M" and len(p) >= 3:
                    cid, dlc, data = p[0] | (p[1] << 8), p[2], bytes(p[3:3 + p[2]])
                    if cid in resp_ids and dlc >= 2 and (data[0] & 0xF0) == 0 and 1 <= data[0] <= 7:
                        seen[cid] = data[1:1 + data[0]]
            time.sleep(0.005)
        return seen


def _ask(tester: Tester, node: str, payload: bytes) -> bytes | None:
    n = NODES[node]
    return tester.request(n["req"], payload, {n["resp"]}).get(n["resp"])


def read_identity(tester: Tester, node: str) -> dict:
    """{'alive': bool, 'version': str|None, 'serial': str|None, 'problems': [...]} for one board."""
    out: dict = {"alive": False, "version": None, "serial": None, "problems": []}
    tp = _ask(tester, node, bytes([0x3E, 0x00]))
    if tp is None:
        out["problems"].append(f"{node}: no UDS response to TesterPresent")
        return out
    out["alive"] = tp == bytes([0x7E, 0x00])
    if not out["alive"]:
        out["problems"].append(f"{node}: unexpected TesterPresent answer {tp.hex()}")
    for did, key in ((0xF195, "version"), (0xF18C, "serial")):
        r = _ask(tester, node, bytes([0x22, did >> 8, did & 0xFF]))
        if r is None:
            out["problems"].append(f"{node}: no UDS response to ReadDataByIdentifier {did:04X}")
        elif r[0] == 0x7F:
            out["problems"].append(f"{node}: DID {did:04X} refused: {NRC.get(r[2] if len(r) > 2 else 0, 'negative response')}")
        elif r[0] == 0x62 and r[1:3] == bytes([did >> 8, did & 0xFF]):
            out[key] = r[3:].decode("ascii", "replace") if key == "version" else r[3:].hex().upper()
        else:
            out["problems"].append(f"{node}: DID {did:04X} answer {r.hex()} is malformed")
    return out


def _numbers(text: str | None) -> str | None:
    m = re.search(r"\d+(?:\.\d+)+", text or "")
    return m.group(0) if m else None


def uds_report(link, inventory: dict, hellos: dict | None = None) -> dict:
    """Read both boards over UDS and compare with the inventory and the USB hello. -> {'boards': {...}, 'problems': [...]}."""
    tester, boards, problems = Tester(link), {}, []
    for node in NODES:
        ident = read_identity(tester, node)
        boards[node] = ident
        problems += ident["problems"]
        want = _numbers((inventory.get(node) or {}).get("firmware"))
        if ident["version"] and want and ident["version"] != want:
            problems.append(f"{node}: UDS says {ident['version']}, the inventory expects {want} (is not the inventory)")
        said = _numbers((hellos or {}).get(node))
        if ident["version"] and said and ident["version"] != said:
            problems.append(f"{node}: UDS says {ident['version']} but the USB hello says {said}: the two disagree")
    return {"boards": boards, "problems": problems}
