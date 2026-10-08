// SpiStress (v2.9.4 diagnostics): find out whether a wire between the ESP32-S3 and the MCP2515 module is faulty, and which.
//
// The MCP2515 runs in internal LOOPBACK mode (no bus needed). Each SPI line fails in its own way:
//   SO/MISO (GPIO13)  reads wrong, writes right, isolated bit errors
//   SI/MOSI (GPIO11)  writes wrong, reads right; a corrupted instruction can become RTS -> spurious transmit
//   SCK     (GPIO12)  reads AND writes wrong, everything after the glitch shifted by one bit ("tail" errors)
//   CS      (GPIO10)  commands cut short / bytes run as new instructions -> spurious transmits, mode/config changes
//   VCC/GND           chip resets: configuration falls back to its reset defaults
//   errors at 10 MHz but none at 1 MHz: signal integrity (poor GND return, long wires), not an open contact
// TXB2 holds a "canary" frame (ID 0x7FF) that is never requested: if it is ever received, something on SPI
// requested it, which is the stale-frame replay seen in HiL (v2.9.3).
// 'N' over serial switches to NORMAL mode: B then transmits TXB0 (ID 0x100) every 500 us on the real bus (A ACKs and
// forwards), so the transceiver draws its real current pulses; TXB1 (0x200) and the TXB2 canary must never appear on A.
// Output: one text line every 500 ms with this window's counts (see scripts/spi_stress.py).
#include <SPI.h>

const int CS = 10, INT_PIN = 4, MOSI_P = 11, MISO_P = 13, SCK_P = 12;
const uint32_t FAST = 10000000, SLOW = 1000000;   // FAST = the autowp-mcp2515 library's clock
const uint8_t PAT0[8] = {0x00, 0xFF, 0xAA, 0x55, 0x0F, 0xF0, 0x33, 0xCC};
uint8_t PAT[8] = {0x00, 0xFF, 0xAA, 0x55, 0x0F, 0xF0, 0x33, 0xCC};   // normal mode: bytes 0-1 carry a sequence number
uint16_t seq = 0;

struct Win {
  uint32_t it, rd_fast, rd_slow, wr_fast, unstable, tail, iso, resets, cfg, mode, sptx[3];
  uint32_t cls[5];       // got 0xFF, got 0x00, one-bit shift, single bit flip, other
  uint32_t rbits[8], wbits[8];
} w;
bool normal_mode = false;

static inline void beg(uint32_t hz) { SPI.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0)); digitalWrite(CS, LOW); }
static inline void fin() { digitalWrite(CS, HIGH); SPI.endTransaction(); }
uint8_t rd(uint32_t hz, uint8_t a) { beg(hz); SPI.transfer(0x03); SPI.transfer(a); uint8_t v = SPI.transfer(0); fin(); return v; }
void rdn(uint32_t hz, uint8_t a, uint8_t *b, int n) { beg(hz); SPI.transfer(0x03); SPI.transfer(a); for (int i = 0; i < n; i++) b[i] = SPI.transfer(0); fin(); }
void wr(uint32_t hz, uint8_t a, uint8_t v) { beg(hz); SPI.transfer(0x02); SPI.transfer(a); SPI.transfer(v); fin(); }
void wrn(uint32_t hz, uint8_t a, const uint8_t *b, int n) { beg(hz); SPI.transfer(0x02); SPI.transfer(a); for (int i = 0; i < n; i++) SPI.transfer(b[i]); fin(); }
void bitmod(uint32_t hz, uint8_t a, uint8_t m, uint8_t v) { beg(hz); SPI.transfer(0x05); SPI.transfer(a); SPI.transfer(m); SPI.transfer(v); fin(); }

bool setup_chip() {
  beg(SLOW); SPI.transfer(0xC0); fin();   // RESET
  delay(10);
  wr(SLOW, 0x2A, 0x00); wr(SLOW, 0x29, 0x90); wr(SLOW, 0x28, 0x82);   // CNF1..3: 500 kbit/s at 8 MHz, as the library
  const uint8_t h0[5] = {0x20, 0x00, 0, 0, 8}, h1[5] = {0x40, 0x00, 0, 0, 8}, h2[5] = {0xFF, 0xE0, 0, 0, 1};
  wrn(SLOW, 0x31, h0, 5); wrn(SLOW, 0x36, PAT, 8);    // TXB0: ID 0x100, the read pattern
  wrn(SLOW, 0x41, h1, 5);                              // TXB1: ID 0x200, the write-test buffer
  wrn(SLOW, 0x51, h2, 5); wr(SLOW, 0x56, 0xC5);        // TXB2: canary ID 0x7FF
  wr(SLOW, 0x60, 0x60); wr(SLOW, 0x70, 0x60);          // RXB0/1: receive any
  wr(SLOW, 0x2C, 0x00);                                // CANINTF clear
  bitmod(SLOW, 0x0F, 0xE0, normal_mode ? 0x00 : 0x40);  // REQOP = normal or loopback
  delay(5);
  return (rd(SLOW, 0x0E) >> 5) == (normal_mode ? 0 : 2);
}

void classify(uint8_t e, uint8_t g, uint32_t *bits) {
  uint8_t x = e ^ g;
  for (int b = 0; b < 8; b++) if ((x >> b) & 1) bits[b]++;
  if (g == 0xFF) w.cls[0]++;
  else if (g == 0x00) w.cls[1]++;
  else if (g == (uint8_t)(e << 1) || g == (uint8_t)((e << 1) | 1) || g == (e >> 1) || g == ((e >> 1) | 0x80)) w.cls[2]++;
  else if (__builtin_popcount(x) == 1) w.cls[3]++;
  else w.cls[4]++;
}

// compare a burst; "tail" = every byte from the first error on is wrong (framing/clock), else isolated
void check(const uint8_t *e, const uint8_t *g, uint32_t *count, uint32_t *bits) {
  int first = -1, bad = 0;
  for (int i = 0; i < 8; i++) if (e[i] != g[i]) { if (first < 0) first = i; bad++; classify(e[i], g[i], bits); }
  if (first < 0) return;
  (*count)++;
  if (bad == 8 - first && bad > 1) w.tail++; else w.iso++;
}

void line_test() {
  int pins[2] = {INT_PIN, MISO_P};
  const char *names[2] = {"INT GPIO4", "SO GPIO13"};
  for (int cs = 1; cs >= 0; cs--) {
    digitalWrite(CS, cs);
    delayMicroseconds(50);
    for (int i = 0; i < 2; i++) {
      pinMode(pins[i], INPUT_PULLUP); delay(2); int up = digitalRead(pins[i]);
      pinMode(pins[i], INPUT_PULLDOWN); delay(2); int dn = digitalRead(pins[i]);
      pinMode(pins[i], INPUT);
      Serial.printf("L CS=%d %s up=%d down=%d %s\n", cs, names[i], up, dn, up == dn ? "driven" : "FLOATING");
    }
  }
  digitalWrite(CS, HIGH);
}

uint32_t t_win = 0, t_cfg = 0, t_tx = 0, n_tx = 0;
void setup() {
  Serial.begin(921600);
  delay(1200);
  pinMode(CS, OUTPUT); digitalWrite(CS, HIGH);
  pinMode(INT_PIN, INPUT);
  line_test();
  SPI.begin(SCK_P, MISO_P, MOSI_P, CS);
  bool ok = setup_chip();
  Serial.printf("H SpiStress 1.0 loopback=%s\n", ok ? "ok" : "FAILED");
  memset(&w, 0, sizeof w);
  t_win = millis();
}

void loop() {
  uint8_t g[8], g2[8], v[8];
  if (Serial.available()) {
    int c = Serial.read();
    if (c == 'N' || c == 'L') {
      normal_mode = c == 'N';
      memcpy(PAT, PAT0, 8); seq = 0;
      Serial.printf("H mode=%s ok=%d\n", normal_mode ? "normal" : "loopback", setup_chip());
    }
  }
  if (normal_mode && micros() - t_tx >= 500) {
    t_tx = micros();
    if (!(rd(FAST, 0x30) & 0x08)) {   // TXB0 free: stamp the next sequence number, then RTS TXB0
      seq++;
      PAT[0] = (uint8_t)seq; PAT[1] = (uint8_t)(seq >> 8);
      wrn(FAST, 0x36, PAT, 2);
      beg(FAST); SPI.transfer(0x81); fin(); n_tx++;
    }
  }
  w.it++;
  rdn(FAST, 0x36, g, 8); check(PAT, g, &w.rd_fast, w.rbits);
  rdn(SLOW, 0x36, g, 8); check(PAT, g, &w.rd_slow, w.rbits);
  for (int i = 0; i < 8; i++) v[i] = (uint8_t)esp_random();
  wrn(FAST, 0x46, v, 8);
  rdn(SLOW, 0x46, g, 8); rdn(SLOW, 0x46, g2, 8);
  if (memcmp(g, g2, 8) != 0) w.unstable++;
  else check(v, g, &w.wr_fast, w.wbits);

  uint8_t intf = rd(SLOW, 0x2C);
  if (intf & 0x03) {   // something was transmitted (in loopback it comes straight back): nothing was requested
    for (int b = 0; b < 2; b++) {
      if (!(intf & (1 << b))) continue;
      uint8_t sidh = rd(SLOW, b ? 0x71 : 0x61);
      w.sptx[sidh == 0xFF ? 2 : (sidh == 0x40 ? 1 : 0)]++;
    }
    wr(SLOW, 0x2C, 0x00);
  }
  uint32_t now = millis();
  if (now - t_cfg >= 20) {
    t_cfg = now;
    uint8_t c2 = rd(SLOW, 0x29), c3 = rd(SLOW, 0x28), om = rd(SLOW, 0x0E) >> 5;
    if (c2 == 0x00 && c3 == 0x00) { w.resets++; setup_chip(); }        // back to reset defaults: the chip reset
    else if (c2 != 0x90 || c3 != 0x82) { w.cfg++; setup_chip(); }      // configuration overwritten
    else if (om != (normal_mode ? 0 : 2)) { w.mode++; setup_chip(); }                       // operating mode changed
    else {
      rdn(SLOW, 0x36, g, 8);
      if (memcmp(g, PAT, 8) != 0) { wrn(SLOW, 0x36, PAT, 8); }        // a corrupted write hit the pattern: restore
    }
  }
  if (now - t_win >= 500) {
    t_win = now;
    Serial.printf("S %lu it=%lu rf=%lu rs=%lu wf=%lu un=%lu tail=%lu iso=%lu cls=%lu,%lu,%lu,%lu,%lu "
                  "rb=%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu wb=%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu tx=%lu,%lu,%lu rst=%lu cfg=%lu mode=%lu sent=%lu tec=%u\n",
                  now, w.it, w.rd_fast, w.rd_slow, w.wr_fast, w.unstable, w.tail, w.iso,
                  w.cls[0], w.cls[1], w.cls[2], w.cls[3], w.cls[4],
                  w.rbits[0], w.rbits[1], w.rbits[2], w.rbits[3], w.rbits[4], w.rbits[5], w.rbits[6], w.rbits[7],
                  w.wbits[0], w.wbits[1], w.wbits[2], w.wbits[3], w.wbits[4], w.wbits[5], w.wbits[6], w.wbits[7],
                  w.sptx[0], w.sptx[1], w.sptx[2], w.resets, w.cfg, w.mode, n_tx, rd(SLOW, 0x1C));
    n_tx = 0;
    memset(&w, 0, sizeof w);
  }
}
