# Switch on board B's own Wi-Fi radio (v2.28)

Board B (the safety controller, an ESP32-S3) can receive its firmware updates over your Wi-Fi instead of the USB cable. This page
is the whole set-up, in the order to do it. **You type the passwords in your own terminal; nothing here asks you to paste a
password into a chat, a file or the repo.**

## What you need, and where each thing comes from

| Thing | Where it comes from |
|---|---|
| Wi-Fi name and password | Your own router. Printed on its label, or in its admin page. It must be the **2.4 GHz** network (the ESP32-S3 has no 5 GHz). Use your main network, not the guest network (guest networks usually stop devices talking to your PC). A phone hotspot set to 2.4 GHz also works if this PC joins it too. |
| This PC's address on that network | `python -m fleet.broker addresses` (step 3). Give this PC a fixed address in the router ("DHCP reservation") so it does not change. |
| Broker user name and password | **You make them up** (step 2). They are not from any service. 8+ characters. |
| Board id | You choose; the default `board-b` is fine. It is only a name in the MQTT topics. |

## Where the passwords end up

| Where | What | In git? |
|---|---|---|
| The board's flash (NVS), via USB | Wi-Fi name and password, broker address, user and password | no, it is on the board |
| `secrets/broker_passwd` on this PC | the broker login as an **argon2 hash**, never the password | no, `secrets/` is git-ignored |
| The firmware images in `fleet/images/` | nothing: no secret is compiled in | not tracked |
| Your terminal session | the environment variables, only while that terminal is open | no |

The board stores the Wi-Fi password in plain form in its flash. Anyone holding the board and a USB cable can read it out. That is
acceptable for a bench; a product would encrypt the flash.

## Steps

All commands are PowerShell, run from the bench's top folder (the one with `run.py` in it).

**0. Put the Wi-Fi firmware on board B (once, by cable).** The firmware running on B now has no Wi-Fi code, so this one step needs
the cable. It also resets B to the 2.10 baseline the matrix starts from.

```powershell
.venv\Scripts\python.exe scripts\hil_flash.py --b COM13
```

**1. Check the network.** Make sure this PC and the board will be on the same 2.4 GHz network, and that Windows calls it
**Private** (Settings, Network and internet, Wi-Fi, your network, Network profile type: Private).

**2. Make the broker login.** It asks for a password twice, hidden:

```powershell
.venv\Scripts\python.exe -m fleet.broker login --user ssb
```

**3. Find this PC's address:**

```powershell
.venv\Scripts\python.exe -m fleet.broker addresses
```

The first line is normally right (for example `192.168.1.20`).

**4. Let the board in through the Windows firewall.** Once, in a PowerShell **run as Administrator**. Private networks only:

```powershell
New-NetFirewallRule -DisplayName "ssb-mqtt" -Direction Inbound -Protocol TCP -LocalPort 1884 -Action Allow -Profile Private
```

**5. Start the broker** in its own terminal (port 1884, because another broker such as Mosquitto often owns the standard 1883) and leave it running:

```powershell
.venv\Scripts\python.exe -m fleet.broker serve
```

**6. Give the board its settings** (another terminal; close anything else that has COM13 open). It asks for each item; the two
passwords are typed hidden:

```powershell
.venv\Scripts\python.exe scripts\provision_wifi.py --port COM13
```

It then waits up to 45 s and prints `OK: the board is on the network and connected to the broker.` To check any time, with no
passwords shown: `scripts\provision_wifi.py --port COM13 --info`. To try your entries against the board's limits without sending
anything: `--dry-run`.

**7. Run the update tests over Wi-Fi.** Stop the broker from step 5 first (the test starts its own on the same port). The matrix asks for the broker
password, hidden:

```powershell
.venv\Scripts\python.exe -m fleet.ota_matrix --real COM13 --wifi
```

It resets the board over the USB cable (an EN-pin pulse is test equipment, not part of an update) and sends every image over
Wi-Fi. A full REL-01..10 run pushes an image of about 1 MB several times; plan on 10 to 25 minutes. When it ends 10/10,
`python -m fleet.release` counts it as evidence.

**To forget the network:** `scripts\provision_wifi.py --port COM13 --clear`. With nothing provisioned the radio never switches on.

## If it does not connect

| Symptom | Likely cause |
|---|---|
| "did not join the network" | 5 GHz network; wrong name or password (names are case-sensitive); a guest network; WPA3-only network (use WPA2/WPA3 mixed) |
| Joined, but "not on the broker" | the broker is not running (step 5); the address is not this PC's; user or password differ from step 2; the firewall rule is missing, or the network is not "Private" |
| Joined and on the broker, but the matrix times out | something else is using port 1884 (an old broker): `Get-NetTCPConnection -LocalPort 1884` |
| "no answer to the network set-up message" | board B is not running the Wi-Fi firmware: redo step 0 |
| It worked, then the board vanished | the radio is **off while a scenario runs** by design, and returns 2 s after it ends |

## What to know before you rely on it

- **Plain MQTT, no TLS.** The signature protects the *image*: someone on your network can stop an update, not forge one. Do not
  expose the broker to the internet or to a network you do not trust. TLS with a pinned certificate and one login per board is the
  next step.
- **The radio is a guest on a safety controller.** It runs in its own tasks, is off while a scenario runs, and a board with
  nothing provisioned never starts it. The release report lists these limits.
- **The firmware grew from about 350 KB to about 1 MB** (the network stack), 78 % of the 1.25 MB update slot, so an update takes
  roughly three times as long as before and there is limited room left.
- Credentials are accepted **over USB only**, never over the network they configure.
