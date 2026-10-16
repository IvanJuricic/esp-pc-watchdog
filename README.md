# esp-pc-watchdog

Remotely monitor and control a PC from Telegram, using an ESP32. Message a
Telegram bot to wake the machine, check its health, or shut it down — the ESP32
sits on your network and does the work, so it keeps running even when the PC is
off.

## Features

- **Wake-on-LAN** — power the PC on with `/wake`
- **Health reporting** — `/status` pulls a full snapshot (CPU/GPU/NVMe temps,
  load, RAM, GPU usage and VRAM, disk space, NVMe wear, failed systemd units,
  container count) from an agent running on the PC
- **Up/down check** — `/ping` does a quick TCP probe of the PC's SSH port
- **Graceful shutdown** — `/shutdown` requests a poweroff, with a 60-second
  `/confirm` step to prevent accidents
- **Self-diagnostics** — `/diag` runs every connectivity check in sequence and
  reports where the chain breaks, so faults can be found without a serial cable
- **Device health** — `/watchdog` reports the ESP32's own uptime, heap, and last
  reset reason
- Access is restricted to a single authorised Telegram chat ID
- Hardware task watchdog auto-restarts the ESP32 if it hangs, and the firmware
  restarts itself after repeated poll failures

## How it works

The ESP32 connects to WiFi and long-polls the Telegram Bot API for commands.
When an authorised user sends a command, the device acts on it:

- `/wake` broadcasts a Wake-on-LAN magic packet to the PC's MAC address.
- `/ping` and `/status` open TCP connections to the PC (port 22 for the up/down
  check; port 9999 for the stats/shutdown agent).
- `/status` fetches a JSON stats document from the PC agent over HTTP and
  formats it into a readable report.
- `/shutdown` calls the agent's shutdown endpoint, guarded by a confirmation
  window and a shared token.

The companion PC-side agent (an HTTP server on port 9999 exposing
`/stats/<token>` and `/shutdown/<token>`) is **not** part of this repository —
it runs on the target PC and must be provided separately.

## Requirements

- An ESP32 board
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/) v5.x
- A Telegram bot (create one via [@BotFather](https://t.me/BotFather)) and your
  numeric chat ID
- A PC with Wake-on-LAN enabled in BIOS/UEFI, and the companion stats/shutdown
  agent listening on port 9999

## Configuration

All settings live in `#define`s at the top of `main/pc_watchdog.c`:

| Setting | Meaning |
|---|---|
| `WIFI_SSID` / `WIFI_PASS` | Your WiFi network |
| `BOT_TOKEN` | Telegram bot token from BotFather |
| `CHAT_ID` | Your Telegram chat ID (the only ID allowed to issue commands) |
| `STATS_TOKEN` / `SHUTDOWN_TOKEN` | Shared secrets that must match the PC agent |
| `PC_MAC` / `PC_IP` | Target PC's MAC (for WoL) and IP address |
| `BROADCAST_IP` | Your subnet's broadcast address (for the WoL packet) |
| `PC_HTTP_PORT` | Port the PC agent listens on (default 9999) |

> **Do not commit real credentials.** The values are blank in this repo on
> purpose. Fill them in locally, but keep your bot token and WiFi password out
> of git history — anything pushed to a public repo is effectively public
> forever. A simple approach: keep your filled-in values in a local-only header
> that git ignores (e.g. add `main/secrets.h` to `.gitignore` and `#include` it),
> rather than editing the committed defaults directly.

## Build and flash

```bash
# Set up the ESP-IDF environment for the current shell
. $IDF_PATH/export.sh

# Set the target (first time only)
idf.py set-target esp32

# Build, flash, and watch the serial log
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Replace `/dev/ttyUSB0` with your serial port. Exit the monitor with `Ctrl+]`.

The firmware logs verbosely over serial (tokens are masked), and `/diag` reports
the same checks over Telegram — so you can troubleshoot with or without a cable.

## Telegram commands

| Command | Action |
|---|---|
| `/status` | Full PC health report |
| `/ping` | Quick up/down check |
| `/wake` | Send Wake-on-LAN packet |
| `/shutdown` | Power off the PC (asks for `/confirm`) |
| `/diag` | Run all connectivity checks and report each result |
| `/watchdog` | Report the ESP32's own health |
| `/help` | List commands |

## Project structure

```
esp-pc-watchdog/
├── main/
│   ├── pc_watchdog.c      # firmware: WiFi, Telegram polling, commands, WoL
│   └── CMakeLists.txt
├── CMakeLists.txt         # top-level ESP-IDF project file
├── sdkconfig              # generated ESP-IDF configuration
└── .gitignore
```