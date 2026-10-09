"""v2.29: the design points of board A's receive path, pinned in the source (the behaviour itself needs the boards: scripts/overflow_by_scenario.py)."""
from __future__ import annotations

import re
import unittest
from pathlib import Path

SRC = (Path(__file__).resolve().parent.parent / "hil" / "firmware" / "BusNode" / "BusNode.ino").read_text(encoding="utf-8")


class ReceivePathTests(unittest.TestCase):
    def test_the_can_interrupt_is_installed_from_the_can_task_not_from_setup(self):
        """A GPIO interrupt is serviced on the core that installed it. setup() runs on core 1; the CAN task is pinned to core 0."""
        task = SRC[SRC.index("static void can_task("):SRC.index("void setup()")]
        setup = SRC[SRC.index("void setup()"):]
        self.assertIn("attachInterrupt(", task)
        self.assertNotIn("attachInterrupt(", setup)
        self.assertRegex(SRC, r"xTaskCreatePinnedToCore\(can_task,[^;]*, 0\);")

    def test_the_receive_buffer_is_read_in_one_transaction(self):
        self.assertIn("0x90", SRC)
        self.assertIn("read_rxb0_fast(&f)", SRC)
        self.assertNotIn("mcp.readMessage(MCP2515::RXB0", SRC)
        fast = SRC[SRC.index("static bool read_rxb0_fast"):SRC.index("// v2.9.4: receive from RXB0 only.")]
        self.assertEqual(fast.count("SPI.beginTransaction("), 1)

    def test_rollover_stays_off_and_only_rxb0_is_read(self):
        """v2.9.4: the MCP2515 sometimes misreports RXB1; the design reads RXB0 only (rollover off in mcp_init)."""
        self.assertIn("RXB0 only", SRC)

    def test_the_diagnostics_message_carries_the_latency_fields_the_pc_parses(self):
        self.assertRegex(SRC, r"uint8_t d\[111\]")
        hil = (Path(__file__).resolve().parent.parent / "ssb" / "hil.py").read_text(encoding="utf-8")
        for n in ("len(p) >= 67", "len(p) >= 87", "len(p) >= 111", "rx_overflow_run"):
            self.assertIn(n, hil)

    def test_every_overflow_is_filed_under_a_cause_or_under_none(self):
        self.assertTrue(re.search(r"if \(!any\) ovf_by\[4\]\+\+;", SRC))


if __name__ == "__main__":
    unittest.main()
