#!/usr/bin/python3

# beebo: patches the downloaded RadioLib before each build, until upstream has
# the fix. SX126x::sleep() waits delay(1) after SetSleep; on ESP32 that is
# vTaskDelay(1), which can end almost at once. The chip ignores SPI for ~500 us
# after SetSleep (SX1261/2 datasheet 13.1.1), so a wake-up sent sooner is lost,
# the chip stays asleep and every later command waits out the 1 s BUSY timeout.

import os

Import("env")

PATCHES = [(
    os.path.join("src", "modules", "SX126x", "SX126x_commands.cpp"),
    "  // wait for SX126x to safely enter sleep mode\n  this->mod->hal->delay(1);\n",
    "  // wait for SX126x to safely enter sleep mode\n  this->mod->hal->delayMicroseconds(600);\n",
)]

libdir = os.path.join(env.subst("$PROJECT_LIBDEPS_DIR"), env.subst("$PIOENV"), "RadioLib")
for rel, old, new in PATCHES:
    path = os.path.join(libdir, rel)
    with open(path) as f:
        text = f.read()
    if new in text:
        continue
    if old not in text:
        raise SystemExit(f"patch-radiolib.py: {rel} changed upstream, patch does not apply")
    with open(path, "w") as f:
        f.write(text.replace(old, new, 1))
    print(f"patch-radiolib.py: patched {rel}")
