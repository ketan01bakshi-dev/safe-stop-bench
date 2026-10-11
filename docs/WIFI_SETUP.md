# Board B's own Wi-Fi radio, with your Mosquitto and TLS (v2.30)

Board B (the safety controller, an ESP32-S3) receives its firmware updates over your Wi-Fi instead of the USB cable. The link is
**encrypted (TLS)**, the board **only trusts a broker certificate signed by a private CA that you make and pin on the board**, and it
logs in with a user name and password that only exist inside that encrypted link. **You type the passwords in your own terminal;
nothing here asks you to paste a password into a chat, a file or the repo.**

```
board B  --Wi-Fi 2.4 GHz-->  your router  -->  this PC (192.168.1.6)
                                                 Mosquitto service
                                                   port 1883  anonymous, localhost use      (your other projects: unchanged)
                                                   port 8883  TLS + login, topics ssb/# only (the boards)
```

## Where each thing comes from, and where it ends up

| Thing | Comes from | Ends up |
|---|---|---|
| Wi-Fi name and password | your router (its label or admin page). Must be the **2.4 GHz** network; not a guest network | the board's flash only |
| This PC's address | `192.168.1.6` today. **Reserve it in the router** (DHCP reservation): the certificate names this address | inside the broker certificate and the board's settings |
| Broker login | **you make it up** (8+ characters), in step 2 | hashed in `C:\Program Files\Mosquitto\ssb\passwd`; the board's flash |
| CA certificate `ca.pem` (public) | `python -m fleet.certs make` | the board's flash, and the PC test tools |
| CA key `ca.key` (SECRET) | the same command | `secrets\tls\` on this PC, git-ignored. Never copied anywhere; only needed to issue a new broker certificate |
| Broker certificate and key `server.pem`, `server.key` | the same command | `secrets\tls\` and `C:\Program Files\Mosquitto\ssb\` (the key readable by SYSTEM and Administrators only) |

## Steps

All commands are PowerShell, run from the bench's top folder (the one with `run.py` in it), as yourself unless it says Administrator.

**0. Wi-Fi firmware on board B (done).** B runs the 2.10 baseline with the Wi-Fi/TLS firmware and has nothing provisioned. To redo it:
`.\.venv\Scripts\python.exe scripts\hil_flash.py --b COM13`.

**1. Certificates (done for 192.168.1.6).** They exist in `secrets\tls\`. Look: `.\.venv\Scripts\python.exe -m fleet.certs show`. If this PC's
address ever changes, make new ones (every board then needs the new `ca.pem`, step 3):
`.\.venv\Scripts\python.exe -m fleet.certs make --host <new address> --force`.

**2. Add the boards' listener to Mosquitto. Open PowerShell as Administrator**, go to the bench folder, and first look without changing anything:

```powershell
.\scripts\setup_mosquitto_tls.ps1 -DryRun
```

Then, when you are happy with the plan:

```powershell
.\scripts\setup_mosquitto_tls.ps1
```

It backs up `mosquitto.conf`, copies the certificates, asks you for the **broker password** (twice, hidden), writes the access rules,
adds one marked block to the config, restarts the service (your port-1883 clients reconnect after a few seconds), **restores your old
config by itself if the service fails to start**, and adds a Windows Firewall rule for TCP 8883 on Private networks only. Run it again
any time: the block is replaced, not repeated.

**3. Give board B its settings** (close anything else that uses COM13):

```powershell
.\.venv\Scripts\python.exe scripts\provision_wifi.py --port COM13
```

It asks for, in this order: the network name (type it exactly as your router shows it), the Wi-Fi password (hidden), "encrypt with TLS?"
(answer `1`), this PC's address (accept `192.168.1.6`), port (accept `8883`), user (`ssb`), the **broker password from step 2** (hidden), board id
(accept `board-b`). It uploads the CA certificate first, then the settings, then waits up to 45 seconds and prints
`OK: the board is on the network and connected to the broker`. Check any time, with no password shown: `--info`.

**4. Run the update tests over Wi-Fi with TLS:**

```powershell
.\.venv\Scripts\python.exe -m fleet.ota_matrix --real COM13 --wifi --broker 127.0.0.1:8883 --tls-ca
```

It asks for the broker password (hidden), checks the broker's certificate against `ca.pem` as the PC side of the test, resets the board over the USB
cable (test equipment) and sends the images over Wi-Fi. Expect about 15 minutes (measured: each full push takes 70 to 100 seconds); it prints each result as it finishes and a progress line every 500 chunks. After a pass, `python -m fleet.release` counts it as evidence
and the standing limit changes from "no encryption" to the smaller list under "What it does not protect against".

**To forget everything on the board:** `scripts\provision_wifi.py --port COM13 --clear` (the radio then never starts).

## What the board checks

1. It joins the Wi-Fi network you gave it.
2. It opens TLS to the address in its settings and demands a certificate **signed by the pinned CA** and **naming that address**. A broker with any other
   certificate (including a real, publicly signed one) is refused: the board carries no public CA list.
3. Only then does it send its login. The password never travels in the clear.
4. Mosquitto lets that login read and write `ssb/#` and nothing else.
5. TLS with no CA stored is refused rather than downgraded: the radio simply stays off.

## What it does not protect against (and what would)

- **The certificates start in 1970.** The ESP32 has no clock; after power-up it thinks it is 1 January 1970, and TLS rejects a certificate that is "not yet
  valid". So the board cannot notice an expired certificate (they end in 2049). *Fix:* give the board a time source (NTP) and issue certificates with real dates.
- **One login for every board.** *Fix:* a login per board, each limited to its own `ssb/ota/<id>/#`.
- **The CA key lives on this PC.** Anyone who gets `secrets\tls\ca.key` can make a broker the boards will trust. Keep that folder private (it is git-ignored).
- **Credentials in the board's flash are not encrypted.** Someone holding the board and a USB cable can read the Wi-Fi password out. *Fix:* flash encryption.
- The signature check on each firmware image is the real guard: an attacker who defeats the link still cannot install an image the key did not sign.

## If it does not connect

| Symptom | Likely cause |
|---|---|
| "did not join the network" | 5 GHz network; wrong name or password (case-sensitive); guest network; WPA3-only (use WPA2 or mixed) |
| Joined, "not on the broker" | firewall rule missing (step 2 adds it; check `Get-NetFirewallRule -DisplayName ssb-mqtt-tls`); network profile not **Private**; the address in the settings is not this PC's |
| Joined, broker refuses | wrong user or password; or the certificate does not name the address the board connects to (this PC's address changed): `fleet.certs show`, then redo steps 1 to 3 |
| The service did not start after step 2 | the script restored your old config; look at the message it printed. Your backup is `mosquitto.conf.bak-<time>` |
| `--info` says "TLS ON BUT NO CA STORED" | provisioning was interrupted: run `scripts\provision_wifi.py` again |
| "no answer to the network set-up message" | board B is not running the Wi-Fi firmware: step 0 |
| Worked, then the board vanished | by design the radio is **off while a scenario runs** and returns 2 s after it ends |

## Notes

- **Mosquitto 2.1 warnings.** Start-up logs two deprecation notices: `per_listener_settings` goes away in Mosquitto 3.0 (where password and ACL plugins
  take over), and your own `allow_anonymous true` line has a newer spelling. Both still work; nothing here changed your 1883 listener.
- **Only board B has a radio.** B receives updates; A (the bus observer) has nothing to update and its CAN receive timing matters (a late wake loses a frame;
  see `CHANGES_V2.md`, v2.29), and the Wi-Fi stack shares its processor core. Putting A on the network would be separate work.
- **The firmware is about 1 MB** (78 % of the 1.25 MB update slot), so a push is about 5150 chunks.
- **Without Mosquitto:** `python -m fleet.broker login` / `serve` runs a small stand-alone broker on port 1884 (plain MQTT, for a first test or CI) and
  `fleet.ota_matrix --real COM13 --wifi` starts one by itself. Use the TLS route above for anything you leave running.
