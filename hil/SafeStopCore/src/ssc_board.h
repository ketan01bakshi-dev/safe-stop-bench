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
const uint32_t LINK_BAUD = 921600;

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
        return true;
      }
    }
  }
  *xtal = "none";
  return false;
}
}  // namespace board
#endif
