"""How the HiL firmware is compiled: the one place for the flags every build of SafetyNode/BusNode needs (scripts/hil_flash.py, fleet/image.py,
health/preflight.py all use it).

LINK_FLAGS (v2.34): esp-mqtt creates its task unpinned, and an unpinned task that lands on core 1 during a TLS handshake starves the 10 ms control loop for
130-190 ms. hil/SafeStopCore/src/ssc_esp_pin.cpp pins it to core 0, but only if the linker routes xTaskCreatePinnedToCore calls to that file. arduino-cli
ignores a library's `ldflags`, so the flag goes on the command line. A build without it still works: the board's own check (INFO flag bit 5) then refuses to keep
the MQTT client running, instead of running it where it can stall the loop.
"""
from __future__ import annotations

LINK_FLAGS = "-Wl,--wrap=xTaskCreatePinnedToCore"


def build_properties(cflags: str = "") -> list[str]:
    """The `--build-property` arguments for arduino-cli: our link flags, plus extra compiler flags if any."""
    args = ["--build-property", f"compiler.c.elf.extra_flags={LINK_FLAGS}"]
    if cflags:
        args += ["--build-property", f"compiler.cpp.extra_flags={cflags}"]
    return args
