# JetFan Controller

[→ فارسی](README-fa.md)

A smart temperature-based fan controller for JetFans (up to 13,500 RPM). An Arduino handles PWM output while a Python daemon on the PC reads CPU/GPU temperatures and adjusts fan speed in real time.

## Changelog: v1 → v2

| Feature | v1 (manual) | v2 (automatic) |
|---------|------------|----------------|
| Control | Manual via serial commands `0-4` + potentiometer | Fully automatic via PC temperature sensing |
| Temperature | None | Reads CPU (`sysfs`) + NVIDIA GPU (`nvidia-smi`) |
| Algorithm | Discrete 5-step PWM | Continuous temperature curve + PID across full range |
| Smart step-down | ❌ | Below 50°C, gradually reduces speed by ~500 RPM every 10s |
| Asymmetric response | ❌ | Fast on temp rise, slow on temp fall (pot-controlled) |
| Sensitivity pot | Direct PWM control | Adjusts PID aggressiveness (0=slow, 255=instant) |
| PC communication | None | Bidirectional serial protocol (`PWM`, `SENS`, `RPM`) |
| Fallback | N/A | 3s serial timeout → pot takes over manual control |
| Auto-reconnect | N/A | Detects Arduino via `/dev/serial/by-id/`, resilient to port changes |
| Logging | Serial monitor | Rotating file + journald + one-line status |

## How It Works

```
┌─────────────┐  PWM:xxx  ┌──────────────┐
│   Arduino   │◄──────────│  PC Daemon   │
│  (PWM out)  │──────────►│  (Python)    │
└─────────────┘ SENS/RPM  └──────┬───────┘
                                 │
                          ┌──────┴───────┐
                          │  Temperatures │
                          │  CPU + GPU    │
                          └──────────────┘
```

- **Arduino** reads the sensitivity potentiometer (A0), sends it to the PC, and applies the PWM command it receives.
- **PC daemon** reads CPU temp (`/sys/class/thermal/`) and NVIDIA GPU temp (`nvidia-smi`), computes optimal PWM via a temperature curve + PID, and sends it over serial.
- **Manual fallback:** if the Arduino stops receiving commands for 3 seconds, the potentiometer directly controls PWM — you still have full manual control.
- **Asymmetric response:** fan speeds up fast on rising temperature, slows down gradually on falling temperature.
- **Sensitivity potentiometer** adjusts how aggressively the system reacts (0 = sluggish averaging, 255 = instant response).
- **Smart step-down:** below 50°C and stable, fan speed gradually drops by ~500 RPM every 10 seconds to find the quietest viable speed.
- **Emergency:** if any temperature exceeds 85°C, fan goes 100% immediately.

## Installation

### 1. Upload Arduino Sketch

Open `main.ino` in the Arduino IDE, select your board and port, and upload.

### 2. Install PC Daemon

The daemon auto-detects the Arduino via `/dev/serial/by-id/usb-Arduino*` and auto-reconnects on cable changes.

```bash
chmod +x enable.sh
./enable.sh
```

This installs dependencies and registers the daemon as a systemd **user service** that starts at boot via `loginctl enable-linger`.

### 3. Disable / Remove

```bash
./disable.sh
```

## Requirements

- **Hardware:** Arduino (any board with Serial), JetFan with PWM + tachometer, 10kΩ potentiometer
- **Software:** Python 3, `pyserial`, `nvidia-smi` (optional, for GPU temp)
- **OS:** Linux (sysfs thermal zones for CPU temp)

## Pinout

| Component | Arduino Pin |
|-----------|-------------|
| Fan PWM   | D9          |
| Fan Tach (in)  | D2 (interrupt) |
| Tach passthrough | D7     |
| Potentiometer | A0      |
| LED (built-in) | D13   |

## Protocol

| Direction | Format | Description |
|-----------|--------|-------------|
| PC → Arduino | `PWM:xxx\n` | Set PWM (0–255) |
| Arduino → PC | `SENS:xxx\n` | Potentiometer sensitivity |
| Arduino → PC | `RPM:xxx\n` | Measured fan RPM |
| Arduino → PC | `MODE:...\n` | Mode change event |

## Temperature Curve

| Temp | PWM | ~RPM |
|------|-----|------|
| 35°C | 20  | 1058 |
| 45°C | 45  | 2382 |
| 55°C | 80  | 4235 |
| 65°C | 120 | 6353 |
| 75°C | 175 | 9265 |
| 85°C | 255 | 13500 (max) |

## Logging

Rotating logs at `logs/jetfan.log` (max 5 × 1MB).  
One-line status at `logs/jetfan-latest.txt`.

```bash
tail -f logs/jetfan.log
```
