// The board's own Wi-Fi radio and MQTT client (v2.28, integration plan P4): the OTA messages over the network instead of the USB cable.
//
// Nothing is compiled into the image: the network name, passwords and broker address are PROVISIONED over USB into the board's flash
// (NVS namespace "net") by scripts/provision_wifi.py. The signed images in fleet/images/ are copied to every board and sit on disks, so a
// password inside them would be copied too. A board with nothing provisioned never switches the radio on.
//
//   PC -> B (USB)   'N' <op> ...        B -> PC  'n' <op> <status> ...
//     0x01 SET    port:u16 LE, then six NUL-terminated strings: ssid, wifi password, broker host, broker user, broker password, device id
//     0x02 CLEAR  forget everything (and switch the radio off)
//     0x03 INFO   answers flags (provisioned / wifi up / mqtt up), rssi, then ssid, ip, host, device id. NEVER a password.
//
//   Over the network, the same bytes the USB link carries (the OTA 'U' messages in, the 'u' answers and a 'H' hello out):
//     ssb/ota/<device id>/to_board     PC -> B        ssb/ota/<device id>/from_board    B -> PC
//   The hello is published on every (re)connect, after the subscription, so a PC that waits for it never talks to a deaf board.
//
// The radio is a guest on a safety controller: it is OFF while a scenario runs (poll(allowed=false)) and comes back 2 s after the run
// ends, it runs in its own tasks (Wi-Fi and esp-mqtt), and the 10 ms control loop only ever pops a queue. Plain MQTT, no TLS: the
// signature check protects the IMAGE, not the link. See docs/WIFI_SETUP.md.
#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <mqtt_client.h>
#include <ssc_core.h>

namespace net {

enum Op : uint8_t { SET = 1, CLEAR = 2, INFO = 3 };
enum Status : uint8_t { OK = 0, BAD_MESSAGE = 1, TOO_LONG = 2, NO_STORE = 3 };

static char ssid[33], wpass[64], host[65], user[33], mpass[65], dev[17];
static uint16_t port = 1883;
static bool provisioned = false;

struct Msg {
  uint16_t n;
  uint8_t d[280];
};
static QueueHandle_t rxq = nullptr;
static esp_mqtt_client_handle_t cli = nullptr;
static char t_to[48], t_from[48], client_id[40];
static volatile bool mqtt_up = false, need_hello = false;
static bool radio_on = false;
static uint32_t allowed_since = 0;

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
  p.end();
  provisioned = ssid[0] && host[0] && dev[0];
}

static void begin() {
  rxq = xQueueCreate(6, sizeof(Msg));
  load();
}

static void rep(uint8_t op, uint8_t status, const uint8_t *extra = nullptr, uint8_t n = 0) {
  uint8_t p[120] = {op, status};
  if (n > 100) n = 100;
  if (n) memcpy(p + 2, extra, n);
  uint8_t buf[130];
  size_t k = ssc::frame_msg('n', p, (uint8_t)(2 + n), buf);
  Serial.write(buf, k);
}

static void stop_radio() {
  if (cli) {
    esp_mqtt_client_stop(cli);
    esp_mqtt_client_destroy(cli);
    cli = nullptr;
  }
  mqtt_up = false;
  if (radio_on) {
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);
    radio_on = false;
  }
}

static void on_event(void *, esp_event_base_t, int32_t id, void *data) {
  auto *e = (esp_mqtt_event_handle_t)data;
  if (id == MQTT_EVENT_CONNECTED) {
    mqtt_up = true;
    esp_mqtt_client_subscribe_single(cli, t_to, 1);
    need_hello = true;
  } else if (id == MQTT_EVENT_DISCONNECTED) {
    mqtt_up = false;
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
  cfg.broker.address.transport = MQTT_TRANSPORT_OVER_TCP;
  cfg.credentials.client_id = client_id;
  if (user[0]) {
    cfg.credentials.username = user;
    cfg.credentials.authentication.password = mpass;
  }
  cfg.session.keepalive = 20;
  cfg.network.reconnect_timeout_ms = 2000;
  cfg.buffer.size = 1024;
  cfg.buffer.out_size = 1024;
  cli = esp_mqtt_client_init(&cfg);
  if (!cli) return;
  esp_mqtt_client_register_event(cli, MQTT_EVENT_ANY, on_event, nullptr);
  esp_mqtt_client_start(cli);
}

// Call from loop(). `allowed` is false while a scenario runs. `hello` is the same text the USB hello carries.
static void poll(bool allowed, const char *hello) {
  if (!provisioned) return;
  uint32_t now = millis();
  if (!allowed) {
    allowed_since = 0;
    if (radio_on) stop_radio();
    return;
  }
  if (!allowed_since) allowed_since = now ? now : 1;
  if (now - allowed_since < 2000) return;
  if (!radio_on) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);   // power-save adds tens of ms of latency to every message
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, wpass);
    radio_on = true;
  }
  if (!cli && WiFi.status() == WL_CONNECTED) start_mqtt();
  if (need_hello && mqtt_up && cli) {
    uint8_t buf[100];
    size_t k = ssc::frame_msg('H', (const uint8_t *)hello, (uint8_t)min((size_t)90, strlen(hello)), buf);
    need_hello = false;
    esp_mqtt_client_enqueue(cli, t_from, (const char *)buf, (int)k, 1, 0, true);
  }
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
  if (cli && mqtt_up) esp_mqtt_client_enqueue(cli, t_from, (const char *)d, (int)n, 1, 0, true);
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
      Preferences pr;
      if (!pr.begin("net", false)) return rep(SET, NO_STORE);
      pr.putString("ssid", s);
      pr.putString("wpass", wp);
      pr.putString("host", h);
      pr.putString("user", u);
      pr.putString("mpass", mp);
      pr.putString("dev", d);
      pr.putUShort("port", pt);
      pr.end();
      stop_radio();
      allowed_since = 0;
      load();
      return rep(SET, OK);
    }
    case CLEAR: {
      Preferences pr;
      if (pr.begin("net", false)) {
        pr.clear();
        pr.end();
      }
      stop_radio();
      memset(ssid, 0, sizeof ssid);
      memset(wpass, 0, sizeof wpass);
      memset(host, 0, sizeof host);
      memset(user, 0, sizeof user);
      memset(mpass, 0, sizeof mpass);
      memset(dev, 0, sizeof dev);
      provisioned = false;
      return rep(CLEAR, OK);
    }
    case INFO: {
      uint8_t x[100];
      x[0] = (uint8_t)((provisioned ? 1 : 0) | (radio_on && WiFi.status() == WL_CONNECTED ? 2 : 0) | (mqtt_up ? 4 : 0));
      x[1] = (uint8_t)(int8_t)(radio_on && WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
      String ip = (radio_on && WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("");
      int k = snprintf((char *)x + 2, sizeof x - 2, "%s%c%s%c%s%c%s", ssid, 0, ip.c_str(), 0, host, 0, dev);
      return rep(INFO, OK, x, (uint8_t)min(98, k + 2));
    }
    default: return rep(p[0], BAD_MESSAGE);
  }
}

}  // namespace net
