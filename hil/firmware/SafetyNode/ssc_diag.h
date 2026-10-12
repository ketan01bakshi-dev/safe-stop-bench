// Board B loop-stall diagnostics (v2.33). Compiled in ONLY with -DB_DIAG=1 (scripts/hil_flash.py --cflags=-DB_DIAG=1): a normal build contains none of
// this (every macro below is empty), so the release firmware is unchanged.
//
// Why: board B's 10 ms control loop was seen to stop for ~190 ms, with the radio on and nothing running, and then to catch up with a burst of
// ~19 cycles whose frames overran the MCP2515's three transmit buffers (failed transmits, frames out of order on the bus). The loop is
// `loop()` on core 1; something blocked it or took the CPU. This records WHICH section of loop() was running when a stall happened, and what the
// Wi-Fi and MQTT stacks were doing, with B's own clock, and reports both over USB every second:
//   'S'  u32 uptime_ms, u32 longest gap between two passes of loop() (us), u32 stalls so far, 8 x u32 longest duration of each section (us),
//        then the last up to 8 stalls: u32 time_ms, u32 duration_us, u8 section (255 = the whole pass, 0..7 = a section of loop())
//   'E'  u32 uptime_ms, u32 events so far, then the last up to 24 events: u32 time_ms, u8 code, i16 argument
// A stall is a section or a pass that took 20 ms or more. Event codes: 1 radio on, 2 radio off, 3 MQTT client started, 4 MQTT up, 5 MQTT down,
// 6 MQTT error (argument = esp-mqtt error type), 7 test operation (argument = which), 8 / 9 NVS write test begin / end, 10 the radio task's
// unused stack in bytes (v2.34; one event per new minimum); 110..117 are the Arduino
// Wi-Fi event numbers themselves (110 STA start, 111 stop, 112 connected, 113 disconnected (argument = reason), 115 got IP, 117 lost IP).
#pragma once
#include <Arduino.h>
#include <ssc_core.h>

#ifdef B_DIAG
#include <esp_freertos_hooks.h>
#include <esp_timer.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <lwip/tcpip.h>
namespace bdiag {

enum Sec : uint8_t { S_SERIAL, S_NETPOLL, S_NETRX, S_PERSIST, S_CONFIRM, S_WD, S_CAN, S_NODE, S_COUNT };
enum Ev : uint8_t { E_RADIO_ON = 1, E_RADIO_OFF = 2, E_MQTT_START = 3, E_MQTT_UP = 4, E_MQTT_DOWN = 5, E_MQTT_ERR = 6, E_TEST = 7, E_NVS_BEGIN = 8, E_NVS_END = 9, E_RADIO_STACK = 10, E_MQTT_PRIO = 11, E_MQTT_AFF = 12, E_LOOP_PRIO = 13 };

static const uint32_t STALL_US = 20000;
struct Stall { uint32_t t_ms, dur_us; uint8_t sec; };
struct Event { uint32_t t_ms; uint8_t code; int16_t arg; };

static uint32_t sec_max_us[S_COUNT];
static Stall stalls[8];
static Event events[24];
static uint32_t n_stalls = 0, n_events = 0, gap_max_us = 0, last_loop_us = 0, last_report_ms = 0, last_n_stalls = 0, last_n_events = 0;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static void event(uint8_t code, int16_t arg = 0) {   // called from the Wi-Fi and MQTT tasks as well
  portENTER_CRITICAL(&mux);
  Event &e = events[n_events % 24];
  e.t_ms = millis();
  e.code = code;
  e.arg = arg;
  n_events++;
  portEXIT_CRITICAL(&mux);
}

// Who was running on each core, every millisecond (a FreeRTOS tick hook per core): when a stall is detected, the last 256 ms of both cores are
// summarised as text ('T' frame): task name and number of ticks, and the longest gap between two ticks (a core with interrupts masked has gaps).
struct Tick { uint32_t us; char nm[6]; };
static Tick ticks[2][256];
static volatile uint32_t tick_n[2];
static char tick_text[260] = "";
static uint32_t tick_texts = 0, last_tick_texts = 0;

static void IRAM_ATTR tick_note(int core) {
  Tick &t = ticks[core][tick_n[core] & 255];
  t.us = (uint32_t)esp_timer_get_time();
  const char *n = pcTaskGetName(xTaskGetCurrentTaskHandle());
  memcpy(t.nm, n, 5);
  t.nm[5] = 0;
  tick_n[core] = tick_n[core] + 1;
}
static void IRAM_ATTR tick0() { tick_note(0); }
static void IRAM_ATTR tick1() { tick_note(1); }

// A task on core 0 (high priority, wakes every ms) that records the state of loopTask (0 running, 1 ready, 2 blocked, 3 suspended), and logs the
// real priority and core affinity of the esp-mqtt task and of loopTask whenever the esp-mqtt task appears.
struct Obs { uint32_t us; uint8_t st; uint8_t cur, base; bool core_lock; };
static Obs obs[256];
static volatile uint32_t obs_n;
static void observer(void *) {
  TaskHandle_t lt = nullptr, last_mt = nullptr;
  for (;;) {
    vTaskDelay(1);
    if (!lt) lt = xTaskGetHandle("loopTask");
    TaskHandle_t mt = xTaskGetHandle("mqtt_task");
    if (lt) {
      Obs &o = obs[obs_n & 255];
      o.us = (uint32_t)esp_timer_get_time();
      o.st = (uint8_t)eTaskGetState(lt);
      o.cur = o.base = 0;
      o.core_lock = false;
      if (mt) {
        TaskStatus_t ti;
        vTaskGetInfo(mt, &ti, pdFALSE, eRunning);
        o.cur = (uint8_t)ti.uxCurrentPriority;
        o.base = (uint8_t)ti.uxBasePriority;
        o.core_lock = lock_tcpip_core && xSemaphoreGetMutexHolder(lock_tcpip_core) == mt;
      }
      obs_n = obs_n + 1;
    }
    {
      if (mt != last_mt) {
        last_mt = mt;
        if (mt && lt) {
          BaseType_t a = xTaskGetAffinity(mt);
          event(E_MQTT_PRIO, (int16_t)uxTaskPriorityGet(mt));
          event(E_MQTT_AFF, a == tskNO_AFFINITY ? (int16_t)-1 : (int16_t)a);
          event(E_LOOP_PRIO, (int16_t)uxTaskPriorityGet(lt));
        }
      }
    }
  }
}

static void summarise(uint32_t dur_us) {
  uint32_t from = (uint32_t)esp_timer_get_time() - dur_us - 4000;
  size_t k = snprintf(tick_text, sizeof tick_text, "stall %lu ms | ", (unsigned long)(dur_us / 1000));
  for (int c = 1; c >= 0; c--) {
    char names[6][6];
    uint16_t cnt[6] = {0};
    int nn = 0;
    uint32_t gap = 0, newer = 0, n = tick_n[c];
    for (uint32_t j = 0; j < 256 && j < n; j++) {
      const Tick &t = ticks[c][(n - 1 - j) & 255];
      if ((int32_t)(t.us - from) < 0) break;
      if (j && newer - t.us > gap) gap = newer - t.us;
      newer = t.us;
      int i = 0;
      while (i < nn && strncmp(names[i], t.nm, 5) != 0) i++;
      if (i == nn && nn < 6) {
        memcpy(names[nn], t.nm, 6);
        nn++;
      }
      if (i < 6) cnt[i]++;
    }
    k += snprintf(tick_text + k, sizeof tick_text - k, "core %d:", c);
    for (int i = 0; i < nn; i++) k += snprintf(tick_text + k, sizeof tick_text - k, " %s %u", names[i], cnt[i]);
    k += snprintf(tick_text + k, sizeof tick_text - k, " (gap %lu us) | ", (unsigned long)gap);   // "core 1: loopT 6 mqtt_ 133 (gap 1003 us) | "
  }
  uint16_t st[4] = {0}, boosted = 0, lockheld = 0;
  uint8_t maxcur = 0;
  uint32_t on = obs_n;
  for (uint32_t j = 0; j < 256 && j < on; j++) {
    const Obs &o = obs[(on - 1 - j) & 255];
    if ((int32_t)(o.us - from) < 0) break;
    if (o.st < 4) st[o.st]++;
    if (o.cur > o.base) boosted++;
    if (o.cur > maxcur) maxcur = o.cur;
    if (o.core_lock) lockheld++;
  }
  snprintf(tick_text + k, sizeof tick_text - k, "loop run/ready/blocked %u/%u/%u | mqtt boosted %u (max %u) | lwIP lock %u", st[0], st[1], st[2], boosted, maxcur, lockheld);
  tick_texts++;
}

static void stall(uint8_t sec, uint32_t dur_us) {
  summarise(dur_us);
  Stall &s = stalls[n_stalls % 8];
  s.t_ms = millis();
  s.dur_us = dur_us;
  s.sec = sec;
  n_stalls++;
}

struct Scope {   // times one section of loop()
  uint8_t i;
  uint32_t t0;
  explicit Scope(uint8_t i_) : i(i_), t0(micros()) {}
  ~Scope() {
    uint32_t d = micros() - t0;
    if (d > sec_max_us[i]) sec_max_us[i] = d;
    if (d >= STALL_US) stall(i, d);
  }
};

static void loop_start() {
  static bool hooked = false;
  if (!hooked) {
    hooked = true;
    esp_register_freertos_tick_hook_for_cpu(tick0, 0);
    esp_register_freertos_tick_hook_for_cpu(tick1, 1);
    xTaskCreatePinnedToCore(observer, "bdiag", 3072, nullptr, 22, nullptr, 0);
  }
  uint32_t now = micros();
  if (last_loop_us) {
    uint32_t g = now - last_loop_us;
    if (g > gap_max_us) gap_max_us = g;
    if (g >= STALL_US) stall(255, g);
  }
  last_loop_us = now;
}

static void put32(uint8_t *p, uint8_t &k, uint32_t v) {
  memcpy(p + k, &v, 4);
  k += 4;
}

static void send_frame(uint8_t kind, const uint8_t *p, uint8_t n) {
  uint8_t buf[260];
  size_t k = ssc::frame_msg(kind, p, n, buf);
  Serial.write(buf, k);
}

static void report() {
  uint32_t now = millis();
  if (now - last_report_ms < 1000 && n_stalls == last_n_stalls && n_events == last_n_events) return;
  last_report_ms = now;
  last_n_stalls = n_stalls;
  last_n_events = n_events;
  uint8_t p[130], k = 0;
  put32(p, k, now);
  put32(p, k, gap_max_us);
  put32(p, k, n_stalls);
  for (int i = 0; i < S_COUNT; i++) put32(p, k, sec_max_us[i]);
  uint32_t cnt = n_stalls < 8 ? n_stalls : 8;
  for (uint32_t j = 0; j < cnt; j++) {
    const Stall &s = stalls[(n_stalls - cnt + j) % 8];
    put32(p, k, s.t_ms);
    put32(p, k, s.dur_us);
    p[k++] = s.sec;
  }
  send_frame('S', p, k);
  if (tick_texts != last_tick_texts) {
    last_tick_texts = tick_texts;
    send_frame('T', (const uint8_t *)tick_text, (uint8_t)min((size_t)200, strlen(tick_text)));
  }
  uint8_t q[190];
  k = 0;
  portENTER_CRITICAL(&mux);
  put32(q, k, now);
  put32(q, k, n_events);
  cnt = n_events < 24 ? n_events : 24;
  for (uint32_t j = 0; j < cnt; j++) {
    const Event &e = events[(n_events - cnt + j) % 24];
    put32(q, k, e.t_ms);
    q[k++] = e.code;
    memcpy(q + k, &e.arg, 2);
    k += 2;
  }
  portEXIT_CRITICAL(&mux);
  send_frame('E', q, k);
}

}  // namespace bdiag

#define BD_SEC(i) bdiag::Scope bd_scope_(i)
#define BD_LOOP_START() bdiag::loop_start()
#define BD_REPORT() bdiag::report()
#define BD_EVENT(code, arg) bdiag::event((code), (arg))
#else
#define BD_SEC(i)
#define BD_LOOP_START()
#define BD_REPORT()
#define BD_EVENT(code, arg)
#endif
