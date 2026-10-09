"""OTA over MQTT (P4): the same OtaClient, but the bytes travel through a broker instead of a USB cable.

    OtaClient -> MqttLink --ssb/ota/<dev>/to_board--> broker --> Gateway -> board (emulated, or the real one on a USB port)
              <-          <--ssb/ota/<dev>/from_board--        <--         <-

`MqttLink` has the four calls the client uses on a serial link (write, read, reset_board, close), so nothing in the update
protocol, the image checks or the scenarios changes. The `Gateway` stands where a Wi-Fi radio would stand: it subscribes for a
device's bytes and hands them to the board's link, and publishes whatever the board says. Chunks are 200 bytes and the client is
stop-and-wait, so a broker that loses, repeats or delays a message is the same fault the matrix already injects (REL-05).

What this proves and what it does not: the update survives a real broker, real MQTT framing and a real topic layout, with the real
board behind the gateway. What it does not prove: the board's own Wi-Fi radio and MQTT client (the firmware has neither; adding them
needs the network credentials, which are not in this repo).

Topics (QoS 1):  ssb/ota/<dev>/to_board  bytes PC -> board      ssb/ota/<dev>/from_board  bytes board -> PC
                 ssb/ota/<dev>/ctl       "reset" | "advance <s>" | "cut <n>"  ssb/ota/<dev>/ctl_ack  "ok" | "fail" (answers the last ctl)
"""
from __future__ import annotations

import asyncio
import logging
import socket
import threading
import time
from collections.abc import Callable
from typing import Any

import paho.mqtt.client as mqtt

from ssb.hil import frame_msg

for _name in ("amqtt", "transitions", "asyncio"):
    logging.getLogger(_name).setLevel(logging.CRITICAL)


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class LocalBroker:
    """An amqtt broker in a background thread (the same stand-in the fleet bench used for the cloud broker).

    `bind` 127.0.0.1 keeps it on this PC; 0.0.0.0 lets a board on the LAN connect. With a `password_file` (user:hash lines, made by
    `python -m fleet.broker login`) a client must log in; without one the broker is anonymous, which is only for 127.0.0.1."""

    def __init__(self, port: int | None = None, bind: str = "127.0.0.1", password_file: str | None = None):
        self.port = port or free_port()
        self.bind, self.password_file = bind, password_file
        self.loop: asyncio.AbstractEventLoop | None = None
        self._broker: Any = None

    def start(self) -> LocalBroker:
        from amqtt.broker import Broker
        auth: dict = ({"amqtt.plugins.authentication.FileAuthPlugin": {"password_file": str(self.password_file)}} if self.password_file
                      else {"amqtt.plugins.authentication.AnonymousAuthPlugin": {"allow_anonymous": True}})
        config = {"listeners": {"default": {"type": "tcp", "bind": f"{self.bind}:{self.port}"}}, "plugins": auth}
        loop = self.loop = asyncio.new_event_loop()
        ready = threading.Event()

        def run() -> None:
            asyncio.set_event_loop(loop)
            self._broker = Broker(config, loop=loop)
            loop.run_until_complete(self._broker.start())
            ready.set()
            loop.run_forever()

        threading.Thread(target=run, name="ssb-broker", daemon=True).start()
        if not ready.wait(15):
            raise RuntimeError("the MQTT broker did not start")
        return self

    def stop(self) -> None:
        if not self.loop:
            return
        try:
            asyncio.run_coroutine_threadsafe(self._broker.shutdown(), self.loop).result(5)
        except Exception:   # noqa: BLE001
            pass
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.loop = None

    def __enter__(self) -> LocalBroker:
        return self.start()

    def __exit__(self, *exc: object) -> None:
        self.stop()


def topic(device: str, leaf: str) -> str:
    return f"ssb/ota/{device}/{leaf}"


class BrokerRefused(RuntimeError):
    pass


def _client(port: int, host: str, user: str | None = None, password: str | None = None) -> mqtt.Client:
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    if user:
        c.username_pw_set(user, password)
    outcome: list = []
    done = threading.Event()

    def on_connect(_c, _u, _flags, reason_code, _props=None) -> None:
        outcome.append(reason_code)
        done.set()

    c.on_connect = on_connect
    c.connect(host, port, keepalive=30)
    c.loop_start()
    if not done.wait(8.0):
        c.loop_stop()
        raise BrokerRefused(f"the broker at {host}:{port} did not answer the connect")
    if outcome[0].is_failure:
        c.loop_stop()
        raise BrokerRefused(f"the broker at {host}:{port} refused the login ({outcome[0]})")
    return c


def _wait_subscribed(c: mqtt.Client, subs: list[str], timeout_s: float = 5.0) -> None:
    got: set[int] = set()
    mids: list[int] = []
    done = threading.Event()

    def on_sub(_c, _u, mid, _rc, _p=None) -> None:
        got.add(mid)
        if all(m in got for m in mids):
            done.set()

    c.on_subscribe = on_sub
    for t in subs:
        _, mid = c.subscribe(t, qos=1)
        if mid is not None:
            mids.append(mid)
    if mids and all(m in got for m in mids):
        done.set()
    if not done.wait(timeout_s):
        raise RuntimeError(f"subscription to {subs} was not acknowledged")


class MqttLink:
    """The PC side. Looks like a serial link to OtaClient."""

    def __init__(self, device: str, port: int, host: str = "127.0.0.1", user: str | None = None, password: str | None = None,
                 reset_hook: Callable[[], object] | None = None):
        self.device = device
        self.reset_hook = reset_hook   # when the board's update traffic is on the network, its reset still has to come over the cable
        self._buf = bytearray()
        self._lock = threading.Lock()
        self._ack: str | None = None
        self._ack_evt = threading.Event()
        self.published = 0
        self.c = _client(port, host, user, password)
        self.c.on_message = self._on_message
        _wait_subscribed(self.c, [topic(device, "from_board"), topic(device, "ctl_ack")])

    def _on_message(self, _c, _u, msg) -> None:
        if msg.topic.endswith("/from_board"):
            with self._lock:
                self._buf += msg.payload
        elif msg.topic.endswith("/ctl_ack"):
            self._ack = msg.payload.decode()
            self._ack_evt.set()

    def write(self, data: bytes, now_ms: int = 0) -> None:   # now_ms: the serial link's signature; MQTT has no use for it
        self.c.publish(topic(self.device, "to_board"), bytes(data), qos=1)
        self.published += 1

    def read(self) -> bytes:
        with self._lock:
            out, self._buf = bytes(self._buf), bytearray()
        if not out:
            time.sleep(0.002)   # the client polls; do not spin a core while the broker works
        return out

    def control(self, cmd: str, timeout_s: float = 15.0) -> bool:
        self._ack_evt.clear()
        self.c.publish(topic(self.device, "ctl"), cmd.encode(), qos=1)
        return self._ack_evt.wait(timeout_s) and self._ack == "ok"

    def reset_board(self) -> bool:
        if self.reset_hook is not None:
            self.reset_hook()
            return True
        return self.control("reset")

    def advance(self, seconds: float) -> None:
        self.control(f"advance {seconds}")

    def close(self) -> None:
        self.c.loop_stop()
        self.c.disconnect()


class Gateway:
    """Puts one board behind the broker. `target` is an EmuBoard or a SerialLink (anything with write / read / reset_board)."""

    def __init__(self, device: str, target: Any, port: int, host: str = "127.0.0.1", poll_s: float = 0.005,
                 user: str | None = None, password: str | None = None, hello_on_connect: bool = False):
        """`hello_on_connect`: say hello once subscribed, as the real board does (ssc_net.h). With it the gateway is a stand-in for a
        board that is on the network itself, and the matrix's --wifi path can be exercised without hardware."""
        self.device, self.target, self.poll_s = device, target, poll_s
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self.forwarded = 0
        self.c = _client(port, host, user, password)
        self.c.on_message = self._on_message
        _wait_subscribed(self.c, [topic(device, "to_board"), topic(device, "ctl")])
        if hello_on_connect:
            self._publish_hello()
        self._pump = threading.Thread(target=self._pump_loop, name=f"gw-{device}", daemon=True)
        self._pump.start()

    def _publish_hello(self) -> None:
        text = self.target.hello_text()
        self.c.publish(topic(self.device, "from_board"), frame_msg("H", text.encode()), qos=1)

    def _publish_board_output(self) -> None:
        with self.lock:
            data = self.target.read()
        if data:
            self.c.publish(topic(self.device, "from_board"), data, qos=1)

    def _on_message(self, _c, _u, msg) -> None:
        if msg.topic.endswith("/to_board"):
            with self.lock:
                self.target.write(msg.payload, 0)
            self.forwarded += 1
            self._publish_board_output()
        elif msg.topic.endswith("/ctl"):
            cmd = msg.payload.decode().split()
            ok = False
            with self.lock:
                if cmd[:1] == ["reset"]:
                    ok = bool(self.target.reset_board())
                elif cmd[:1] == ["cut"] and hasattr(self.target, "cut_end_after"):
                    self.target.cut_end_after = int(cmd[1])
                    ok = True
                elif cmd[:1] == ["advance"]:
                    secs = float(cmd[1])
                    if hasattr(self.target, "advance"):
                        self.target.advance(secs)
                    else:
                        time.sleep(secs)
                    ok = True
            self._publish_board_output()
            self.c.publish(topic(self.device, "ctl_ack"), b"ok" if ok else b"fail", qos=1)

    def _pump_loop(self) -> None:
        while not self._stop.wait(self.poll_s):
            try:
                self._publish_board_output()
            except Exception:   # noqa: BLE001 - a closing port must not kill the broker session
                pass

    def close(self) -> None:
        self._stop.set()
        self._pump.join(timeout=2)
        self.c.loop_stop()
        self.c.disconnect()
