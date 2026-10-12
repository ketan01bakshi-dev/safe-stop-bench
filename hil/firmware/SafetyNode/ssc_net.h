// The board's own Wi-Fi radio and MQTT client (v2.28, integration plan P4): the OTA messages over the network instead of the USB cable.
//
// Nothing is compiled into the image: the network name, passwords and broker address are PROVISIONED over USB into the board's flash
// (NVS namespace "net") by scripts/provision_wifi.py. The signed images in fleet/images/ are copied to every board and sit on disks, so a
// password inside them would be copied too. A board with nothing provisioned never switches the radio on.
//
//   PC -> B (USB)   'N' <op> ...        B -> PC  'n' <op> <status> ...
//     0x01 SET    port:u16 LE, then six NUL-terminated strings: ssid, wifi password, broker host, broker user, broker password, device id,
//                 then one flags byte (bit0 = use TLS)
//     0x02 CLEAR  forget everything (and switch the radio off)
//     0x03 INFO   answers flags (b0 provisioned, b1 wifi up, b2 mqtt up, b3 TLS on, b4 CA stored, b5 MQTT refused: see mqtt_pin_state), rssi, free heap and lowest free heap
//                 (u32 each), CA length (u16), then ssid, ip, host, device id. NEVER a password.
//     0x04 CA_BEGIN  total:u16        0x05 CA_CHUNK  offset:u16 + up to 200 bytes        0x06 CA_END   (check + store the CA certificate)
//   TLS (v2.30): the CA certificate (public) is uploaded with CA_*, kept in flash, and PINNED: the broker must present a certificate
//   signed by exactly that CA, for the address in SET. With TLS on and no CA stored, the radio stays off rather than fall back to plain MQTT.
//
//   Over the network, the same bytes the USB link carries (the OTA 'U' messages in, the 'u' answers and a 'H' hello out):
//     ssb/ota/<device id>/to_board     PC -> B        ssb/ota/<device id>/from_board    B -> PC
//   The hello is published on every (re)connect, after the subscription, so a PC that waits for it never talks to a deaf board.
//
// The radio is a guest on a safety controller: it is OFF while a scenario runs (poll(allowed=false)) and comes back 2 s after the run
// ends. It runs entirely off core 1, where the 10 ms control loop runs: the Wi-Fi tasks and the esp-mqtt task (pinned to core 0 by
// hil/SafeStopCore/src/ssc_esp_pin.cpp) and, since v2.34, a "radio" task pinned to core 0 that makes every slow call: WiFi.begin ~50 ms,
// stop_radio 80-130 ms, the MQTT client start. The control loop only decides whether the radio is wanted (poll() sets a flag), pops a queue and
// publishes replies. Measured on the board (CHANGES_V2.md, v2.33 and v2.34): an esp-mqtt task running on core 1 stalled the loop 130-190 ms per
// TLS handshake, and starting and stopping the radio from loop() stalled it 50-130 ms.
// The cost: "off while a scenario runs" takes effect ~100 ms after the run starts, not before it. The signature check protects the IMAGE;
// TLS (when provisioned) protects the LINK and the login. See docs/WIFI_SETUP.md.
#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <mqtt_client.h>
#include <ssc_core.h>
#include "ssc_diag.h"

namespace net {

// Priority of the esp-mqtt task. The Arduino loop (the 10 ms control loop) runs at priority 1 on core 1. At the esp-mqtt default of 5 the task, when
// it ran on core 1, preempted the loop for the ~190 ms of every TLS handshake (v2.33, measured with -DB_DIAG=1). A lower priority is NOT enough on
// its own: mutex priority inheritance lifts the task to 23 for the whole handshake whatever is configured here (v2.34); that is what the core-0 pin
// (ssc_esp_pin.cpp) is for. This keeps it from outranking the loop in the cases the pin does not cover.
#ifndef NET_MQTT_PRIO
#define NET_MQTT_PRIO 1
#endif

enum Op : uint8_t { SET = 1, CLEAR = 2, INFO = 3, CA_BEGIN = 4, CA_CHUNK = 5, CA_END = 6 };
enum Status : uint8_t { OK = 0, BAD_MESSAGE = 1, TOO_LONG = 2, NO_STORE = 3, BAD_CA = 4 };

static char ssid[33], wpass[64], host[65], user[33], mpass[65], dev[17];
static uint16_t port = 1883;
static bool provisioned = false;
static bool tls = false;
static const size_t CA_MAX = 2000;
static char ca_pem[CA_MAX + 4];      // NUL-terminated PEM: esp-tls wants the length to include the NUL
static uint16_t ca_len = 0;
static char ca_stage[CA_MAX + 4];    // an upload in progress; only a complete, plausible one replaces ca_pem
static uint16_t ca_want = 0, ca_have = 0;

struct Msg {
  uint16_t n;
  uint8_t d[280];
};
static QueueHandle_t rxq = nullptr;
static esp_mqtt_client_handle_t cli = nullptr;
static char t_to[48], t_from[48], client_id[40];
static volatile bool mqtt_up = false, need_hello = false;
static volatile bool mqtt_refused = false; // v2.34: the build lacks the link flag that pins the esp-mqtt task to core 0: the radio is never switched on (see mqtt_pin_state)
static uint32_t cli_started_ms = 0;
static volatile bool radio_on = false;     // written by the radio task, or by handle() while it holds radio_mx
static volatile bool want_radio = false;   // written by loop() (poll, handle): should the radio be on? The radio task makes it so
static uint32_t allowed_since = 0;
static SemaphoreHandle_t radio_mx = nullptr;   // guards cli, radio_on and the credentials/CA the radio task reads while it starts things
static TaskHandle_t radio_task = nullptr;
static const char *hello_text = "";            // the text of the USB hello; constant after setup()

struct Lock {   // radio_mx for one scope
  Lock() { xSemaphoreTake(radio_mx, portMAX_DELAY); }
  ~Lock() { xSemaphoreGive(radio_mx); }
};

static void load() {
  provisioned = false;
  Preferences p;
  if (!p.begin("net", true)) return;
  p.getString("ssid", ssid, sizeof ssid);
  p.getString("wpass", wpass, sizeof wpass);
  p.getString("host", host, sizeof host);
  p.getString("user", user, sizeof user);
  p.getString("mpass", mpass, sizeof mpass);
  p.getString("dev", dev, sizeof dev);
  port = p.getUShort("port", 1883);
  tls = p.getUChar("tls", 0) != 0;
  ca_len = 0;
  size_t n = p.getBytesLength("ca");
  if (n > 0 && n <= CA_MAX) {
    p.getBytes("ca", ca_pem, n);
    ca_pem[n] = 0;
    ca_len = (uint16_t)n;
  }
  p.end();
  provisioned = ssid[0] && host[0] && dev[0];
}

static void begin() {
  rxq = xQueueCreate(6, sizeof(Msg));
  radio_mx = xSemaphoreCreateMutex();
  load();
}

static void rep(uint8_t op, uint8_t status, const uint8_t *extra = nullptr, uint8_t n = 0) {
  uint8_t p[200] = {op, status};
  if (n > 190) n = 190;
  if (n) memcpy(p + 2, extra, n);
  uint8_t buf[210];
  size_t k = ssc::frame_msg('n', p, (uint8_t)(2 + n), buf);
  Serial.write(buf, k);
}

// Caller holds radio_mx.
static void stop_radio() {
  BD_EVENT(bdiag::E_RADIO_OFF, 0);
  if (cli) {
    // esp-mqtt (esp32:esp32 3.3.10) crashes if a client is stopped and destroyed before its task has first run: the task then takes the API lock of a client that
    // no longer exists (found by the build-without-the-pin test, v2.34). Give a young client a moment.
    uint32_t age = millis() - cli_started_ms;
    if (age < 50) vTaskDelay(pdMS_TO_TICKS(50 - age));
    esp_mqtt_client_stop(cli);
    esp_mqtt_client_destroy(cli);
    cli = nullptr;
  }
  mqtt_up = false;
  if (radio_on) {
    radio_on = false;   // first: a reader that checks it (INFO) must not touch a Wi-Fi driver that is going away
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);
  }
}

static void on_event(void *, esp_event_base_t, int32_t id, void *data) {
  auto *e = (esp_mqtt_event_handle_t)data;
  if (id == MQTT_EVENT_CONNECTED) {
    mqtt_up = true;
    BD_EVENT(bdiag::E_MQTT_UP, 0);
    esp_mqtt_client_subscribe_single(cli, t_to, 1);
    need_hello = true;
    if (radio_task) xTaskNotifyGive(radio_task);   // the radio task publishes the hello
  } else if (id == MQTT_EVENT_DISCONNECTED) {
    mqtt_up = false;
    BD_EVENT(bdiag::E_MQTT_DOWN, 0);
#ifdef B_DIAG
  } else if (id == MQTT_EVENT_ERROR) {
    bdiag::event(bdiag::E_MQTT_ERR, e->error_handle ? (int16_t)e->error_handle->error_type : (int16_t)-1);
#endif
  } else if (id == MQTT_EVENT_DATA) {
    bool whole = e->current_data_offset == 0 && e->data_len == e->total_data_len && e->data_len > 0 && e->data_len <= (int)sizeof(Msg::d);
    if (whole && e->topic_len == (int)strlen(t_to) && memcmp(e->topic, t_to, e->topic_len) == 0) {
      Msg m;
      m.n = (uint16_t)e->data_len;
      memcpy(m.d, e->data, e->data_len);
      xQueueSend(rxq, &m, 0);   // a full queue drops the message: the stop-and-wait client resends
    }
  }
}

static void start_mqtt() {
  snprintf(t_to, sizeof t_to, "ssb/ota/%s/to_board", dev);
  snprintf(t_from, sizeof t_from, "ssb/ota/%s/from_board", dev);
  snprintf(client_id, sizeof client_id, "ssb-%s-%08x", dev, (unsigned)ESP.getEfuseMac());
  esp_mqtt_client_config_t cfg = {};
  cfg.broker.address.hostname = host;
  cfg.broker.address.port = port;
  if (tls) {
    cfg.broker.address.transport = MQTT_TRANSPORT_OVER_SSL;
    cfg.broker.verification.certificate = ca_pem;           // the pinned CA: the only authority this board trusts
    cfg.broker.verification.certificate_len = ca_len + 1;   // PEM: includes the terminating NUL
  } else {
    cfg.broker.address.transport = MQTT_TRANSPORT_OVER_TCP;
  }
  cfg.credentials.client_id = client_id;
  if (user[0]) {
    cfg.credentials.username = user;
    cfg.credentials.authentication.password = mpass;
  }
  cfg.task.priority = NET_MQTT_PRIO;
  cfg.session.keepalive = 20;
  cfg.network.reconnect_timeout_ms = 2000;
  cfg.buffer.size = 1024;
  cfg.buffer.out_size = 1024;
  BD_EVENT(bdiag::E_MQTT_START, 0);
  cli = esp_mqtt_client_init(&cfg);
  if (!cli) return;
  esp_mqtt_client_register_event(cli, MQTT_EVENT_ANY, on_event, nullptr);
  esp_mqtt_client_start(cli);
  cli_started_ms = millis();
}

// v2.34: the control loop is safe from a TLS handshake only if the esp-mqtt task is on core 0. esp-mqtt creates it unpinned; hil/SafeStopCore/src/ssc_esp_pin.cpp pins
// it when the build passes ssb/firmware_build.py's LINK_FLAGS. Unpinned it can run on core 1, boosted to priority 23 by mutex inheritance, and starve the loop for
// 130-190 ms (CHANGES_V2.md, v2.34). So before the radio is ever switched on, ask the build: create a throw-away task under the name esp-mqtt gives its task,
// unpinned, and see where the wrap placed it. 1 = pinned to core 0, 0 = not (this build has no link flag: the radio stays off and INFO flag bit 5 says why),
// -1 = could not test (out of memory), try again.
static int mqtt_pin_state() {
  static int known = -1;
  if (known >= 0) return known;
  TaskHandle_t t = nullptr;
  auto probe = [](void *) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    vTaskDelete(nullptr);
  };
  if (xTaskCreatePinnedToCore(probe, "mqtt_task", 2048, nullptr, 1, &t, tskNO_AFFINITY) != pdPASS || !t) return -1;
  known = xTaskGetAffinity(t) == 0 ? 1 : 0;
  xTaskNotifyGive(t);   // the probe deletes itself
  return known;
}

// One pass of the radio task (core 0), with radio_mx held: make the radio match want_radio. This is where the slow calls are.
static void radio_step() {
  if (!want_radio || mqtt_refused) {   // mqtt_refused: no pin in this build, so no network (reboot to try again)
    if (radio_on || cli) stop_radio();
    return;
  }
  int pin = mqtt_pin_state();
  if (pin == 0) mqtt_refused = true;
  if (pin != 1) return;
  if (!radio_on) {
    BD_EVENT(bdiag::E_RADIO_ON, 0);
#ifdef B_DIAG
    static bool hooked = false;
    if (!hooked) {
      hooked = true;
      WiFi.onEvent([](arduino_event_id_t id, arduino_event_info_t info) {
        bdiag::event((uint8_t)id, id == ARDUINO_EVENT_WIFI_STA_DISCONNECTED ? (int16_t)info.wifi_sta_disconnected.reason : (int16_t)0);
      });
    }
#endif
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);   // power-save adds tens of ms of latency to every message
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, wpass);
    radio_on = true;
  }
  if (!cli && WiFi.status() == WL_CONNECTED) start_mqtt();
  if (need_hello && mqtt_up && cli) {
    uint8_t buf[100];
    size_t k = ssc::frame_msg('H', (const uint8_t *)hello_text, (uint8_t)min((size_t)90, strlen(hello_text)), buf);
    need_hello = false;
    esp_mqtt_client_enqueue(cli, t_from, (const char *)buf, (int)k, 1, 0, true);
  }
}

static void radio_main(void *) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));   // woken by poll() when the wish changes, by the MQTT task for the hello, else every 20 ms
    {
      Lock lk;
      radio_step();
    }
#ifdef B_DIAG
    static int16_t last_free = -1;   // the high-water mark only falls: one event per new minimum
    int16_t f = (int16_t)min((UBaseType_t)32000, uxTaskGetStackHighWaterMark(nullptr));
    if (f != last_free) {
      last_free = f;
      bdiag::event(bdiag::E_RADIO_STACK, f);
    }
#endif
  }
}

// loop() side: change what the radio task is asked for. Never blocks.
static void set_want(bool on) {
  if (on && !radio_task) xTaskCreatePinnedToCore(radio_main, "radio", 8192, nullptr, 1, &radio_task, 0);   // made on first need: a board with nothing provisioned never has one
  if (on == want_radio) return;
  want_radio = on;
  if (!on && rxq) xQueueReset(rxq);   // what came in before the radio went away is stale
  if (radio_task) xTaskNotifyGive(radio_task);
}

// handle() only, with radio_mx held: switch the radio off NOW, in the caller's context, and keep it off until poll() has waited 2 s again.
static void halt_locked() {
  want_radio = false;
  allowed_since = 0;
  if (rxq) xQueueReset(rxq);
  stop_radio();
}

// Call from loop(). `allowed` is false while a scenario runs. `hello` is the same text the USB hello carries. Cheap: it only sets a flag.
static void poll(bool allowed, const char *hello) {
  hello_text = hello;
  bool usable = provisioned && !(tls && ca_len == 0);   // TLS requested but no CA stored: stay off, never fall back to plain MQTT
  uint32_t now = millis();
  if (!usable || !allowed) {
    allowed_since = 0;
    set_want(false);
    return;
  }
  if (!allowed_since) allowed_since = now ? now : 1;
  set_want(now - allowed_since >= 2000);
}

static bool pop(uint8_t *out, size_t *n) {
  Msg m;
  if (!rxq || xQueueReceive(rxq, &m, 0) != pdTRUE) return false;
  memcpy(out, m.d, m.n);
  *n = m.n;
  return true;
}

// The OTA module's reply sink for messages that came in over the network.
static void publish(const uint8_t *d, size_t n) {
  // publish, not enqueue: enqueue only stores the reply for the MQTT task, which sends it when it next wakes (up to ~1 s later);
  // the stop-and-wait client then pays that delay on every chunk (measured: 1.3 s per chunk, a full image takes ~2 hours)
  // QoS 0: with QoS 1 the broker answers every reply with a PUBACK, the board's TCP delays its acknowledgement (up to 250 ms), and the
  // broker holds the next chunk back until it arrives. Every message here already has an application-level answer and a resend.
  // The radio task may be switching the radio (it holds radio_mx for up to ~130 ms then): the reply is dropped, the PC's stop-and-wait resends.
  if (!want_radio || xSemaphoreTake(radio_mx, pdMS_TO_TICKS(5)) != pdTRUE) return;
  if (cli && mqtt_up) esp_mqtt_client_publish(cli, t_from, (const char *)d, (int)n, 0, 0);
  xSemaphoreGive(radio_mx);
}

static bool take_str(const uint8_t *p, size_t n, size_t &i, char *dst, size_t cap) {
  size_t k = 0;
  while (i < n && p[i]) {
    if (k + 1 >= cap) return false;
    dst[k++] = (char)p[i++];
  }
  if (i >= n) return false;   // no terminating NUL
  dst[k] = 0;
  i++;
  return true;
}

// 'N' messages from USB only: the credentials are never accepted over the network they configure.
static void handle(const uint8_t *p, uint8_t n) {
  if (n < 1) return rep(0, BAD_MESSAGE);
  switch (p[0]) {
    case SET: {
      char s[33], wp[64], h[65], u[33], mp[65], d[17];
      size_t i = 3;
      if (n < 3 + 6) return rep(SET, BAD_MESSAGE);
      uint16_t pt = (uint16_t)(p[1] | p[2] << 8);
      if (!take_str(p, n, i, s, sizeof s) || !take_str(p, n, i, wp, sizeof wp) || !take_str(p, n, i, h, sizeof h) ||
          !take_str(p, n, i, u, sizeof u) || !take_str(p, n, i, mp, sizeof mp) || !take_str(p, n, i, d, sizeof d))
        return rep(SET, TOO_LONG);
      if (!s[0] || !h[0] || !d[0] || !pt) return rep(SET, BAD_MESSAGE);
      uint8_t flags = i < n ? p[i] : 0;
      Preferences pr;
      if (!pr.begin("net", false)) return rep(SET, NO_STORE);
      pr.putUChar("tls", (flags & 1) ? 1 : 0);
      pr.putString("ssid", s);
      pr.putString("wpass", wp);
      pr.putString("host", h);
      pr.putString("user", u);
      pr.putString("mpass", mp);
      pr.putString("dev", d);
      pr.putUShort("port", pt);
      pr.end();
      Lock lk;
      halt_locked();
      load();
      return rep(SET, OK);
    }
    case CLEAR: {
      Preferences pr;
      if (pr.begin("net", false)) {
        pr.clear();
        pr.end();
      }
      Lock lk;
      halt_locked();
      memset(ssid, 0, sizeof ssid);
      memset(wpass, 0, sizeof wpass);
      memset(host, 0, sizeof host);
      memset(user, 0, sizeof user);
      memset(mpass, 0, sizeof mpass);
      memset(dev, 0, sizeof dev);
      memset(ca_pem, 0, sizeof ca_pem);
      ca_len = ca_want = ca_have = 0;
      tls = false;
      provisioned = false;
      return rep(CLEAR, OK);
    }
    case CA_BEGIN: {
      if (n < 3) return rep(CA_BEGIN, BAD_MESSAGE);
      uint16_t total = (uint16_t)(p[1] | p[2] << 8);
      if (total < 100 || total > CA_MAX) return rep(CA_BEGIN, TOO_LONG);
      ca_want = total;
      ca_have = 0;
      return rep(CA_BEGIN, OK);
    }
    case CA_CHUNK: {
      if (n < 4 || !ca_want) return rep(CA_CHUNK, BAD_MESSAGE);
      uint16_t off = (uint16_t)(p[1] | p[2] << 8);
      size_t len = n - 3;
      if (off != ca_have || off + len > ca_want) return rep(CA_CHUNK, BAD_CA);   // out of order or too long: the PC starts again
      memcpy(ca_stage + off, p + 3, len);
      ca_have = (uint16_t)(off + len);
      return rep(CA_CHUNK, OK);
    }
    case CA_END: {
      bool ok = ca_want && ca_have == ca_want;
      ca_stage[ok ? ca_want : 0] = 0;
      ok = ok && strncmp(ca_stage, "-----BEGIN CERTIFICATE-----", 27) == 0 && strstr(ca_stage, "-----END CERTIFICATE-----") != nullptr;
      if (!ok) {
        ca_want = ca_have = 0;
        return rep(CA_END, BAD_CA);
      }
      Preferences pr;
      if (!pr.begin("net", false)) return rep(CA_END, NO_STORE);
      pr.putBytes("ca", ca_stage, ca_want);
      pr.end();
      Lock lk;
      halt_locked();
      ca_want = ca_have = 0;
      load();
      return rep(CA_END, OK);
    }
    case INFO: {
      uint8_t x[190];
      bool got = xSemaphoreTake(radio_mx, pdMS_TO_TICKS(50)) == pdTRUE;   // not while the radio task is switching the radio: report it as down
      bool up = got && radio_on && WiFi.status() == WL_CONNECTED;
      x[0] = (uint8_t)((provisioned ? 1 : 0) | (up ? 2 : 0) | (mqtt_up ? 4 : 0) | (tls ? 8 : 0) | (ca_len ? 16 : 0) | (mqtt_refused ? 32 : 0));
      x[1] = (uint8_t)(int8_t)(up ? WiFi.RSSI() : 0);
      uint32_t fh = ESP.getFreeHeap(), mh = ESP.getMinFreeHeap();
      memcpy(x + 2, &fh, 4);
      memcpy(x + 6, &mh, 4);
      memcpy(x + 10, &ca_len, 2);
      String ip = up ? WiFi.localIP().toString() : String("");
      if (got) xSemaphoreGive(radio_mx);
      int k = snprintf((char *)x + 12, sizeof x - 12, "%s%c%s%c%s%c%s", ssid, 0, ip.c_str(), 0, host, 0, dev);
      return rep(INFO, OK, x, (uint8_t)min(188, k + 12));
    }
#ifdef B_DIAG
    case 7: {   // test hooks, USB only like every 'N' message: 1 MQTT reconnect, 2 Wi-Fi reconnect, 3 radio off then on, 4 NVS write, 5 100 NVS opens
      uint8_t what = n > 1 ? p[1] : 0;
      bdiag::event(bdiag::E_TEST, what);
      if (what == 1) {
        Lock lk;
        if (cli) esp_mqtt_client_reconnect(cli);
      } else if (what == 2) {
        WiFi.disconnect(false, false);
        WiFi.reconnect();
      } else if (what == 3) {
        Lock lk;
        halt_locked();   // back after the 2 s guard, as after the end of a scenario
      } else if (what == 4) {
        static uint8_t k = 0;
        uint8_t b[64];
        memset(b, ++k, sizeof b);
        bdiag::event(bdiag::E_NVS_BEGIN, 0);
        Preferences pr;
        if (pr.begin("diag", false)) {
          pr.putBytes("x", b, sizeof b);
          pr.end();
        }
        bdiag::event(bdiag::E_NVS_END, 0);
      } else if (what == 5) {
        for (int i = 0; i < 100; i++) {
          Preferences pr;
          if (pr.begin("ssb", true)) pr.end();
        }
      }
      return rep(7, OK);
    }
#endif
    default: return rep(p[0], BAD_MESSAGE);
  }
}

}  // namespace net
