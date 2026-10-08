// SPI probe for the MCP2515 module (bring-up diagnostics, v2.9). Raw register reads, no library.
// Expected after RESET: CANSTAT (0x0E) = 0x80 (configuration mode); CANCTRL (0x0F) = 0x87.
// 0xFF everywhere: MISO open, module unpowered, or SO not reaching GPIO13. 0x00 everywhere: MISO stuck low / CS wrong.
// Also writes/reads back CNF1 (0x2A) to prove MOSI works, and samples INT and MISO idle levels.
#include <SPI.h>
const int CS = 10, INT_PIN = 4, MOSI_P = 11, MISO_P = 13, SCK_P = 12;

uint8_t rd(uint8_t a) {
  digitalWrite(CS, LOW); SPI.transfer(0x03); SPI.transfer(a); uint8_t v = SPI.transfer(0x00); digitalWrite(CS, HIGH); return v;
}
void wr(uint8_t a, uint8_t v) {
  digitalWrite(CS, LOW); SPI.transfer(0x02); SPI.transfer(a); SPI.transfer(v); digitalWrite(CS, HIGH);
}
void probe(uint32_t hz) {
  SPI.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW); SPI.transfer(0xC0); digitalWrite(CS, HIGH);   // RESET
  delay(10);
  uint8_t canstat = rd(0x0E), canctrl = rd(0x0F);
  wr(0x2A, 0x5A); uint8_t cnf1 = rd(0x2A); wr(0x2A, 0xA5); uint8_t cnf1b = rd(0x2A);
  SPI.endTransaction();
  Serial.printf("SPI %7lu Hz: CANSTAT=0x%02X (want 0x80) CANCTRL=0x%02X (want 0x87) CNF1 wrote 5A read 0x%02X, wrote A5 read 0x%02X -> %s\n",
                (unsigned long)hz, canstat, canctrl, cnf1, cnf1b,
                (canstat & 0xE0) == 0x80 && cnf1 == 0x5A && cnf1b == 0xA5 ? "SPI OK" : "SPI FAIL");
}
// Is each input really connected to the module? A driven line keeps its level whatever the internal pull does;
// an open (floating) line follows the pull. MISO is only driven while CS is low (SO is high-Z otherwise).
void line_test() {
  int pins[2] = {INT_PIN, MISO_P};
  const char *names[2] = {"INT  GPIO4 ", "MISO GPIO13"};
  for (int cs = 1; cs >= 0; cs--) {
    digitalWrite(CS, cs);
    delayMicroseconds(50);
    for (int i = 0; i < 2; i++) {
      pinMode(pins[i], INPUT_PULLUP);  delay(2); int up = digitalRead(pins[i]);
      pinMode(pins[i], INPUT_PULLDOWN); delay(2); int dn = digitalRead(pins[i]);
      pinMode(pins[i], INPUT);
      Serial.printf("CS=%d  %s pull-up->%d pull-down->%d : %s\n", cs, names[i], up, dn,
                    up == dn ? (up ? "driven HIGH (connected)" : "driven LOW (connected, or shorted to GND)") : "FLOATING (not connected)");
    }
  }
  digitalWrite(CS, HIGH);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(CS, OUTPUT); digitalWrite(CS, HIGH);
  pinMode(INT_PIN, INPUT);
  pinMode(MISO_P, INPUT);
  Serial.printf("idle levels (before SPI): MISO GPIO13=%d  INT GPIO4=%d (INT idles HIGH on a powered module)\n",
                digitalRead(MISO_P), digitalRead(INT_PIN));
  line_test();
  SPI.begin(SCK_P, MISO_P, MOSI_P, CS);
}
void loop() {
  Serial.println("---- SpiProbe ----");
  probe(100000);
  probe(1000000);
  probe(10000000);
  Serial.printf("INT GPIO4=%d\n", digitalRead(INT_PIN));
  delay(3000);
}
