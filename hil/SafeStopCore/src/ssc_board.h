// Board support for the HiL sketches (Arduino only; empty on the PC build).
// Wiring: ESP32-S3 dev board (EdgeHex ESP32-S3 Pro) + MCP2515/TJA1050 module; see docs/HIL_BRINGUP.md step 1.
#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

namespace board {
const int PIN_CAN_CS = 10, PIN_CAN_INT = 4, PIN_CAN_MOSI = 11, PIN_CAN_MISO = 13, PIN_CAN_SCK = 12;
const int PIN_WD_LINE = 5;      // optional hardware watchdog line: board A GPIO5 -> board B GPIO5 (and a common GND)
// v2.12, optional: board A GPIO6 -> IN of a high-level-trigger relay module whose NORMALLY CLOSED contact is in board B's
// 5 V supply (docs/HIL_POWER_CUT.md). Low (A's default, also while A boots or resets) = relay off = B powered.
const int PIN_RELAY = 6;
const uint32_t LINK_BAUD = 921600;

inline void mcp_rollover_off();
// 500 kbps, normal mode; tries an 8 MHz crystal first, then 16 MHz (as CanNode does). Returns false if SPI or the
// bit timing fails; *xtal names the crystal that worked.
inline bool mcp_init(MCP2515 &m, const char **xtal) {
  pinMode(PIN_CAN_INT, INPUT);
  pinMode(PIN_CAN_CS, OUTPUT);
  digitalWrite(PIN_CAN_CS, HIGH);
  SPI.begin(PIN_CAN_SCK, PIN_CAN_MISO, PIN_CAN_MOSI, PIN_CAN_CS);
  delay(50);
  const CAN_CLOCK clocks[2] = {MCP_8MHZ, MCP_16MHZ};
  const char *names[2] = {"8MHz", "16MHz"};
  for (int c = 0; c < 2; c++) {
    for (int attempt = 0; attempt < 3; attempt++) {
      if (m.reset() != MCP2515::ERROR_OK) {
        delay(100);
        continue;
      }
      delay(10);
      if (m.setBitrate(CAN_500KBPS, clocks[c]) == MCP2515::ERROR_OK && m.setNormalMode() == MCP2515::ERROR_OK) {
        *xtal = names[c];
        mcp_rollover_off();
        return true;
      }
    }
  }
  *xtal = "none";
  return false;
}

// CANSTAT.OPMOD (bits 7:5): 0 = normal. Found on hardware (v2.9.3): a short supply or SPI glitch on a jumper wire can
// reset the MCP2515 into configuration mode (4). It then stops ACKing, and nothing in the firmware noticed. Raw read,
// since the library keeps readRegister private. 0xFF (7) means the module is not answering on SPI at all.
inline uint8_t mcp_read_reg(uint8_t addr, uint32_t hz = 1000000) {
  SPI.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CAN_CS, LOW);
  SPI.transfer(0x03);  // READ
  SPI.transfer(addr);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(PIN_CAN_CS, HIGH);
  SPI.endTransaction();
  return v;
}
inline uint8_t mcp_opmode() { return mcp_read_reg(0x0E) >> 5; }   // CANSTAT
inline void mcp_bitmod(uint8_t addr, uint8_t mask, uint8_t val, uint32_t hz = 1000000) {
  SPI.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CAN_CS, LOW);
  SPI.transfer(0x05);  // BIT MODIFY
  SPI.transfer(addr);
  SPI.transfer(mask);
  SPI.transfer(val);
  digitalWrite(PIN_CAN_CS, HIGH);
  SPI.endTransaction();
}
// Found on hardware (v2.9.4): with rollover on (RXB0CTRL.BUKT, the library default), the MCP2515 sometimes sets
// RX1IF without loading a new frame into RXB1 (14 of 15 RXB1 reads in one test returned the previous RXB1 frame),
// so a receiver forwards a frame from minutes or hours earlier. Rollover off: every frame goes to RXB0, RXB1 is never
// used, and a slow reader loses a frame (counted as RX0OVR) instead of replaying an old one.
inline void mcp_rollover_off() { mcp_bitmod(0x60, 0x04, 0x00); }


// Debounced: one bad read is usually an SPI glitch on a jumper (reads 0xFF, mode 7), not a reset controller, and
// re-initialising on it causes the outage it was meant to catch (found on hardware, v2.9.3). Returns the mode only if
// three reads 200 us apart all disagree with normal; *glitch is set when a bad read did not repeat.
inline uint8_t mcp_opmode_confirmed(bool *glitch) {
  *glitch = false;
  uint8_t om = mcp_opmode();
  for (int i = 0; om != 0 && i < 2; i++) {
    delayMicroseconds(200);
    uint8_t again = mcp_opmode();
    if (again == 0) {
      *glitch = true;
      return 0;
    }
    om = again;
  }
  return om;
}
}  // namespace board
#endif
