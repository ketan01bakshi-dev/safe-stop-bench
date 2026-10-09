// OTA update of the safety controller (v2.26, integration plan P4): signed image, A/B slots, trial boot, health check, rollback.
//
// The ESP32-S3 partition table already has two app slots (ota_0 / ota_1, 1.25 MB each; the firmware uses about 26 %). An update
// is written to the SPARE slot while the running one stays untouched, and the boot slot is switched only after the whole image has
// been verified. Transport here is the bench's USB link ('U' from the PC, 'u' back, stop-and-wait with an ack per chunk); the
// image checks do not depend on the transport, so a Wi-Fi / MQTT push can reuse them unchanged.
//
//   PC -> B  'U' <op> ...                                    B -> PC  'u' <op> <status> ...
//     0x01 BEGIN  size:u32 sha256:32 hmac:32 ver_len:u8 ver     refused while a scenario is running (BUSY: only an idle ECU updates)
//                                                                and for a version that is not newer than the running one (DOWNGRADE)
//     0x02 CHUNK  offset:u32 data (<= 200 bytes)                 ack carries the bytes written so far; a repeated chunk is idempotent
//     0x03 END                                                   SHA-256 of the received bytes == the announced hash, and
//                                                                HMAC-SHA256(key, sha256 | size:u32 | version) == the announced hmac;
//                                                                only then the boot slot is switched and TRIAL is recorded in NVS
//     0x04 REBOOT                                                 only after a successful END
//     0x05 STATUS                                                 state, tries, running slot, last result
//   Trial boot: the first boot of a new image is on probation. It confirms itself (state NONE, "committed") after 3 s of healthy
//   operation. Any reset before that (power cut, crash, watchdog) counts as a second try and rolls back to the previous slot, and so
//   does a failed health check at once. After a rollback the old image comes up in its latched safe state like after any reset (NVS restore).
//
// The key below is a DEMO key compiled into the image. A product keeps it in eFuse / a secure element and signs on a build server.
#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <mbedtls/md.h>
#include <mbedtls/sha256.h>
#include <ssc_core.h>

namespace ota {

static const char KEY[] = "demo-only-ssb-ota-key";
enum Op : uint8_t { BEGIN = 1, CHUNK = 2, END = 3, REBOOT = 4, STATUS = 5 };
enum Status : uint8_t { OK = 0, BUSY = 1, BAD_STATE = 2, BAD_SIZE = 3, NO_SPACE = 4, BAD_OFFSET = 5, WRITE_FAIL = 6,
                        HASH_MISMATCH = 7, BAD_SIGNATURE = 8, END_FAIL = 9, NOT_STAGED = 10, BAD_MESSAGE = 11, DOWNGRADE = 12 };
enum State : uint8_t { NONE = 0, TRIAL = 1, ROLLED_BACK = 2 };

static bool receiving = false, staged = false;
static uint32_t want_size = 0, written = 0;
static uint8_t want_sha[32], want_hmac[32];
static char want_ver[17];
static mbedtls_sha256_context sha_ctx;

static void reply(uint8_t op, uint8_t status, const uint8_t *extra = nullptr, uint8_t n = 0) {
  uint8_t p[64] = {op, status};
  if (n > 60) n = 60;
  if (n) memcpy(p + 2, extra, n);
  uint8_t buf[80];
  size_t k = ssc::frame_msg('u', p, (uint8_t)(2 + n), buf);
  Serial.write(buf, k);
}
static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

static void expected_hmac(const uint8_t *sha, uint32_t size, const char *ver, uint8_t out[32]) {
  uint8_t msg[32 + 4 + 16];
  memcpy(msg, sha, 32);
  put32(msg + 32, size);
  size_t vl = strnlen(ver, 16);
  memcpy(msg + 36, ver, vl);
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), (const uint8_t *)KEY, sizeof KEY - 1, msg, 36 + vl, out);
}
static bool same(const uint8_t *a, const uint8_t *b, size_t n) {   // constant time
  uint8_t d = 0;
  for (size_t i = 0; i < n; i++) d |= a[i] ^ b[i];
  return d == 0;
}

static void abort_update() {
  if (receiving) Update.abort();
  receiving = staged = false;
  written = 0;
}

// ---- NVS: the trial-boot state --------------------------------------------------------------------------------------
static uint8_t get_state(uint8_t *tries = nullptr, uint32_t *prev_addr = nullptr, String *last = nullptr) {
  Preferences p;
  uint8_t st = NONE;
  if (p.begin("ota", true)) {
    st = p.getUChar("st", NONE);
    if (tries) *tries = p.getUChar("tries", 0);
    if (prev_addr) *prev_addr = p.getUInt("prev", 0);
    if (last) *last = p.getString("last", "");
    p.end();
  }
  return st;
}
// Test builds only (-DOTA_CUT_AT=n): stand in for a power cut at the nth persistent write of an update's END, once per build. A reset
// at that boundary leaves flash exactly as a cut there would (every earlier write is committed, no later one has started); what it
// cannot show is a torn single write, which needs the relay (docs/HIL_POWER_CUT.md).
#ifdef OTA_CUT_AT
#ifndef FW_MINOR
#define FW_MINOR 10
#endif
static bool cut_armed = false;
static void cut_here() {
  static uint8_t n = 0;
  if (!cut_armed || ++n != OTA_CUT_AT) return;
  Preferences c;
  if (c.begin("otacut", false)) {
    if (c.getUInt("done", 0) == (uint32_t)OTA_CUT_AT * 1000 + FW_MINOR) { c.end(); return; }   // this build already cut once
    c.putUInt("done", (uint32_t)OTA_CUT_AT * 1000 + FW_MINOR);
    c.end();
  }
  Serial.flush();
  delay(30);
  ESP.restart();
  for (;;) {}
}
#else
static inline void cut_here() {}
#endif

// The state flag goes LAST: TRIAL must never be visible with a stale "prev" (a rollback to slot 0 switches nothing, and the unhealthy
// image would keep running). A cut between two of these writes then leaves the old state, which is always safe.
static void set_state(uint8_t st, uint8_t tries, uint32_t prev, const String &last) {
  Preferences p;
  if (!p.begin("ota", false)) return;
  p.putUInt("prev", prev);
  cut_here();
  p.putUChar("tries", tries);
  cut_here();
  p.putString("last", last);
  cut_here();
  p.putUChar("st", st);
  cut_here();
  p.end();
}

static const esp_partition_t *slot_at(uint32_t addr) {
  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, nullptr);
  const esp_partition_t *found = nullptr;
  for (; it; it = esp_partition_next(it))
    if (esp_partition_get(it)->address == addr) found = esp_partition_get(it);
  esp_partition_iterator_release(it);
  return found;
}

[[noreturn]] static void rollback(uint32_t prev, const String &why) {
  const esp_partition_t *p = slot_at(prev);
  if (p) esp_ota_set_boot_partition(p);
  set_state(ROLLED_BACK, 0, 0, "rolled back: " + why);
  Serial.flush();
  delay(50);
  ESP.restart();
  for (;;) {}
}

// Call early in setup(): a trial image that was reset before it confirmed, or that is unhealthy, goes back to the old slot.
static void boot_check(bool healthy) {
  uint8_t tries;
  uint32_t prev;
  if (get_state(&tries, &prev) != TRIAL) return;
  if (esp_ota_get_running_partition()->address == prev) {
    // The probation was recorded but the boot slot never switched (power lost in between): the old image is still running.
    set_state(NONE, 0, 0, "update not completed (power cut before the slot switch)");
    return;
  }
  tries++;
  set_state(TRIAL, tries, prev, "trial boot " + String(tries));
  if (tries >= 2) rollback(prev, "reset before the new image confirmed itself");
  if (!healthy) rollback(prev, "health check failed");
}
// Call from loop(): after 3 s of healthy operation the trial image is committed.
static void confirm_when_stable(uint32_t now_ms, bool healthy, const char *version) {
  static bool done = false;
  if (done || now_ms < 3000) return;
  done = true;
  uint8_t tries;
  uint32_t prev;
  if (get_state(&tries, &prev) != TRIAL) return;
  if (!healthy) rollback(prev, "health check failed");
  set_state(NONE, 0, 0, String("committed ") + version);
}

// ---- the PC's messages ----------------------------------------------------------------------------------------------
static int minor_of(const char *v) {   // "2.13" -> 13
  const char *dot = strchr(v, '.');
  return dot ? atoi(dot + 1) : -1;
}

static void handle(const uint8_t *p, uint8_t n, bool node_active, const char *running_version) {
  if (n < 1) return reply(0, BAD_MESSAGE);
  switch (p[0]) {
    case BEGIN: {
      if (n < 1 + 4 + 32 + 32 + 1) return reply(BEGIN, BAD_MESSAGE);
      if (node_active) return reply(BEGIN, BUSY);   // a scenario is running: an ECU only updates when idle
      abort_update();
      want_size = get32(p + 1);
      memcpy(want_sha, p + 5, 32);
      memcpy(want_hmac, p + 37, 32);
      uint8_t vl = p[69];
      if (vl > 16 || n < 70 + vl) return reply(BEGIN, BAD_MESSAGE);
      memcpy(want_ver, p + 70, vl);
      want_ver[vl] = 0;
      if (want_size < 1024 || want_size > 0x140000) return reply(BEGIN, BAD_SIZE);
      if (minor_of(want_ver) <= minor_of(running_version)) return reply(BEGIN, DOWNGRADE);   // anti-rollback: only newer versions
      if (!Update.begin(want_size)) return reply(BEGIN, NO_SPACE);
      mbedtls_sha256_init(&sha_ctx);
      mbedtls_sha256_starts(&sha_ctx, 0);
      receiving = true;
      written = 0;
      uint8_t w[4];
      put32(w, 0);
      return reply(BEGIN, OK, w, 4);
    }
    case CHUNK: {
      uint8_t w[4];
      if (!receiving || n < 6) return reply(CHUNK, BAD_STATE);
      uint32_t off = get32(p + 1);
      size_t len = n - 5;
      if (off < written) { put32(w, written); return reply(CHUNK, OK, w, 4); }   // a resent chunk: already have it
      if (off > written) { put32(w, written); return reply(CHUNK, BAD_OFFSET, w, 4); }
      if (written + len > want_size) { put32(w, written); return reply(CHUNK, BAD_SIZE, w, 4); }
      if (Update.write(const_cast<uint8_t *>(p + 5), len) != len) { put32(w, written); return reply(CHUNK, WRITE_FAIL, w, 4); }
      mbedtls_sha256_update(&sha_ctx, p + 5, len);
      written += len;
      put32(w, written);
      return reply(CHUNK, OK, w, 4);
    }
    case END: {
      if (!receiving) return reply(END, BAD_STATE);
      if (written != want_size) return reply(END, BAD_SIZE);
      uint8_t sha[32], mac[32];
      mbedtls_sha256_finish(&sha_ctx, sha);
      mbedtls_sha256_free(&sha_ctx);
      if (!same(sha, want_sha, 32)) { abort_update(); return reply(END, HASH_MISMATCH); }
      expected_hmac(want_sha, want_size, want_ver, mac);
      if (!same(mac, want_hmac, 32)) { abort_update(); return reply(END, BAD_SIGNATURE); }
      const esp_partition_t *running = esp_ota_get_running_partition();
#ifdef OTA_CUT_AT
      cut_armed = true;
#endif
      // The probation goes on record BEFORE the boot slot switches: a cut after the switch with no record would boot an unverified
      // image with no way back. A cut before the switch leaves TRIAL with the old image running, which boot_check() cleans up.
      set_state(TRIAL, 0, running->address, String("staged ") + want_ver);
      if (!Update.end(true)) {   // also checks the image header and switches the boot slot
        set_state(NONE, 0, 0, "update failed: image header check");
        abort_update();
        return reply(END, END_FAIL);
      }
      cut_here();
#ifdef OTA_CUT_AT
      cut_armed = false;
#endif
      receiving = false;
      staged = true;
      return reply(END, OK);
    }
    case REBOOT:
      if (!staged) return reply(REBOOT, NOT_STAGED);
      reply(REBOOT, OK);
      Serial.flush();
      delay(100);
      ESP.restart();
      return;
    case STATUS: {
      uint8_t tries;
      uint32_t prev;
      String last;
      uint8_t st = get_state(&tries, &prev, &last);
      uint8_t x[48] = {st, tries};
      put32(x + 2, esp_ota_get_running_partition()->address);
      size_t l = min((size_t)40, (size_t)last.length());
      memcpy(x + 6, last.c_str(), l);
      return reply(STATUS, OK, x, (uint8_t)(6 + l));
    }
    default: return reply(p[0], BAD_MESSAGE);
  }
}

}  // namespace ota
