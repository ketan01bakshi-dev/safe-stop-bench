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
        self.assertRegex(SRC, r"uint8_t d\[151\]")
        hil = (Path(__file__).resolve().parent.parent / "ssb" / "hil.py").read_text(encoding="utf-8")
        for n in ("len(p) >= 67", "len(p) >= 87", "len(p) >= 111", "len(p) >= 151", "rx_overflow_run"):
            self.assertIn(n, hil)

    def test_an_overflow_flag_is_confirmed_by_a_second_read_before_it_is_counted_or_cleared(self):
        """v2.32: EFLG is sticky, so a real overflow shows on a second read and a corrupted read (0xFF) does not. Only a confirmed one is counted."""
        block = SRC[SRC.index("if (e & (MCP2515::EFLG_RX0OVR"):SRC.index("counting = true;")]
        self.assertIn("bool real = (mcp.getErrorFlags()", block)
        self.assertLess(block.index("bool real"), block.index("overflows++"), "the second read comes first")
        self.assertRegex(block, r"if \(real\) \{\s*ovf_confirmed\+\+;\s*overflows\+\+;")
        self.assertLess(block.index("overflows++"), block.index("mcp.clearRXnOVR()"))
        self.assertIn("ovf_raw[", block, "an unconfirmed read is still recorded")

    def test_an_all_ones_interrupt_flag_read_is_read_again_not_acted_on(self):
        drain = SRC[SRC.index("static void drain_rxb0()"):SRC.index("// A UDS request addressed to A")]
        self.assertRegex(drain, r"if \(intf == 0xFF\) \{[^}]*intf_ff\+\+;\s*continue;")

    def test_every_overflow_is_filed_under_a_cause_or_under_none(self):
        self.assertTrue(re.search(r"if \(!any\) ovf_by\[4\]\+\+;", SRC))


if __name__ == "__main__":
    unittest.main()
