# GIGA R1 — Shelly Pro 3EM + Zendure SolarFlow Monitor & Watchdog

An **independent** monitor and safety watchdog for a home battery setup, running on an
**Arduino GIGA R1 WiFi** with the **Giga Display Shield**. It reads a **Shelly Pro 3EM**
(grid meter) and a **Zendure SolarFlow** battery **read-only** over their local HTTP APIs,
shows live data and a daily energy balance, and raises a visible alarm if the battery
misbehaves.

Built as a diverse, independent cross-check after a Zendure firmware update made the
battery ignore its configured limits (e.g. discharging into the grid). A simple, cheap
board supervising a more complex one — the *checker* is independent of the *doer*.

> **Read-only:** the sketch never writes to the Zendure, so it does not disturb the
> device's own cloud/HEMS control.

## Features

- **Grid power** live: total + per phase L1/L2/L3 (W / V / A)
- **Zendure status** (read-only): SoC, output power, acStatus
- **Daily energy balance** [kWh] since midnight (auto-reset), from the Shelly's real
  energy counters — `+` = net feed-in, `−` = net grid draw
- **Clock** from the Shelly (local, DST-correct)
- **Watchdog / alarm** — red blinking frame + banner on:
  - battery **feeding into the grid while discharging** (HEMS runaway)
  - **deep discharge** (SoC below threshold)
  - **BMS fault**
  with per-type **event counters** (persist until restart)
- **Heartbeat dot** + **hardware watchdog** — the sketch supervises itself and
  auto-recovers from a freeze

## Hardware

- Arduino GIGA R1 WiFi
- Arduino Giga Display Shield (800×480, landscape)
- Shelly Pro 3EM (3-phase grid meter, reachable on the LAN)
- Zendure SolarFlow (e.g. 2400 / 2400 Pro, reachable on the LAN via WiFi)

## Libraries

- **ArduinoJson** (>= 7.x)
- **Arduino_GigaDisplay_GFX**
- **Arduino Mbed OS GIGA Boards** core (provides `mbed` for the hardware watchdog)

## Setup

1. Copy `shelly_monitor_V1_0/arduino_secrets.example.h` →
   `shelly_monitor_V1_0/arduino_secrets.h`
2. Fill in your WiFi credentials and the **local IPs** of your Shelly and Zendure.
3. Open `shelly_monitor_V1_0/shelly_monitor_V1_0.ino` in the Arduino IDE, select the
   **Arduino GIGA R1** board, and upload.

`arduino_secrets.h` is git-ignored — your credentials are never committed.

## Notes on the Shelly API fields

- Power / phases: `EM.GetStatus?id=0` → `total_act_power`, `a_/b_/c_act_power`,
  `_voltage`, `_current`
- Energy totals: `EMData.GetStatus?id=0` → **`total_act`** (import Wh) and
  **`total_act_ret`** (export Wh) — note the totals are *not* named `*_energy`
  (that suffix exists only on the per-phase fields)
- Time: `Sys.GetStatus` → `time` ("HH:MM", local)
- **Sign convention:** the raw Shelly value is inverted (`* -1`), so in this sketch
  **negative = grid draw (Bezug)**, **positive = feed-in (Einspeisung)**. If your CT
  clamps are mounted the other way round, remove the `* -1`.

## ⚠️ Disclaimer

This is a **read-only monitor for display purposes**, *not* a certified safety device.
The real cell protection is the battery's own **BMS**. Any optional contactor-based
emergency cutoff (not part of this sketch) must be wired by a **qualified electrician**.
**Use at your own risk.**

## License

[MIT](LICENSE) — Copyright (c) 2026 Reinhard Jesolowitz
