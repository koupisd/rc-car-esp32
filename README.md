# RC Car — ESP32 + PS3 Controller

Firmware for a 4-wheel-drive RC car built on an ESP32. You drive it with a PS3 controller over Bluetooth. A small speaker plays a synthesized engine sound that revs with the throttle, plus a two-tone horn.

## Features

- 4 TT motors driven by two DRV8833 motor drivers
- The front and rear wheels turn at different speeds to steer the car; 
- Turbo mode: hold R2 for full speed (normal driving is capped)
- Headlights, tail lights, reversing lights, turn indicators and hazard lights
- Engine sound and horn played through a MAX98357A I2S amplifier
- Motors stop automatically if the controller disconnects

## Hardware

| Part | Qty |
|---|---|
| ESP32 dev board (ESP32-WROOM, `esp32dev`) | 1 |
| DRV8833 dual motor driver | 2 |
| TT gear motor | 4 |
| MAX98357A I2S amplifier + speaker | 1 |
| PS3 (Sixaxis / DualShock 3) controller | 1 |
| LEDs + resistors for lights | 5 channels |
| Battery pack | 1 |

## Wiring

### Motors

| Motor | Driver | IN1 | IN2 |
|---|---|---|---|
| Left Front (LF) | DRV8833 #1 | GPIO 14 | GPIO 12 |
| Right Front (RF) | DRV8833 #1 | GPIO 26 | GPIO 25 |
| Left Rear (LR) | DRV8833 #2 | GPIO 32 | GPIO 33 |
| Right Rear (RR) | DRV8833 #2 | GPIO 18 | GPIO 19 |

Connect **nSLEEP** on both DRV8833s to 3.3 V to keep them awake. The ESP32, both drivers and the battery must share a **common GND**.

### Lights

| Function | GPIO |
|---|---|
| Headlights | 4 |
| Tail lights | 5 |
| Reversing lights | 16 |
| Left indicator | 17 |
| Right indicator | 21 |

### MAX98357A amplifier

| MAX98357A | ESP32 |
|---|---|
| LRC (WS) | GPIO 23 |
| BCLK | GPIO 13 |
| DIN | GPIO 15 |
| SD (mute) | GPIO 27 (optional; tie to VIN instead to skip) |
| VIN | 5 V |
| GND | GND |

## Controls

| Input | Action |
|---|---|
| Left stick ↕ | Throttle (forward / reverse) |
| Right stick ↔ | Steering |
| R2 (hold) | Turbo: full speed |
| ✕ Cross (hold) | Horn |
| ▲ Triangle | Headlights on/off |
| ■ Square | Hazard lights on/off |
| L1 | Left indicator on/off |
| R1 | Right indicator on/off |

## Building and uploading

The project uses [PlatformIO](https://platformio.org/) in VS Code.

1. Install VS Code and the **PlatformIO IDE** extension.
2. Clone this repository and open the folder in VS Code.
3. Click **✓ Build** in the PlatformIO toolbar. The first build downloads the ESP32 toolchain and the PS3 library automatically, which takes a few minutes.
4. Connect the ESP32 over USB and click **→ Upload**.
5. Click **🔌 Serial Monitor** (115200 baud) to see status messages.

> **Version note:** the project is pinned to `espressif32@6.9.0`, which uses **Arduino-ESP32 core 2.0.17**. Do not upgrade to core 3.x, because the PS3 Controller Host library and the motor PWM code only work on 2.x.

## Pairing the PS3 controller

A PS3 controller connects only to the Bluetooth address stored inside it, so you have to write the ESP32's address into the controller once.

1. Find your ESP32's Bluetooth MAC address. Upload any sketch that prints it, or read it from the board's label.
2. Put that address in `src/main.cpp`:
   ```cpp
   #define ESP32_BT_MAC   "C4:DE:E2:19:66:26"
   ```
3. Connect the controller to a PC with a Mini-USB cable, open **SixaxisPairTool**, enter the same address and click **Update**.
4. Upload the firmware and press the **PS** button. The controller LEDs flash, then settle on LED 1 when it connects.

## Tuning

You can adjust these values at the top of `src/main.cpp`:

| Setting | Default | Effect |
|---|---|---|
| `DEADZONE` | 10 | Ignores small stick movements |
| `NORMAL_SPEED_LIMIT` | 180 | Top speed without turbo (out of 255) |
| `TURBO_SPEED` | 255 | Top speed with R2 held |
| `STEERING_STRENGTH` | 0.6 | How much the inner front wheel slows when turning (0–1) |
| `INDICATOR_PERIOD_MS` | 500 | Indicator blink rate |
| `ENGINE_FREQ_IDLE` / `ENGINE_FREQ_MAX` | 60 / 180 Hz | Engine pitch at idle and at full throttle |
| `ENGINE_MASTER_VOL` / `HORN_VOL` | 0.70 | Engine and horn volume |

## Project layout

```
├── platformio.ini                  # Board, platform version, libraries
├── src/main.cpp                    # Firmware
└── rc_car_esp32_ps3_audio/         # Original Arduino IDE sketch (reference only)
```
