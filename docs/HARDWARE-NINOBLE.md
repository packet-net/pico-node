# Target hardware: the NinoTNC BLE expansion board (NinoBLE Rev5)

*Written 2026-06-07. pico-node's reference carrier board is now mumrah's
[ninotnc-ble](https://github.com/mumrah/ninotnc-ble) (NinoBLE), Rev5 — an
open-hardware Pico W carrier for a NinoTNC, with an optional OLED and an SD card.
Tom has one built up. This document fixes pico-node's pin map to that board so
his board runs our firmware (we ignore the board's BLE role — pico-node uses the
CYW43's WiFi side, which the BLE firmware leaves alone). Pinout extracted from
the NinoBLE Rev5 firmware (`firmware/config.h`, `main_aprs.c`, `oled.c`,
`sd_config.c`) and `PICO_BOARD pico_w`.*

## The board

- **MCU**: Raspberry Pi **Pico W** (RP2040 + CYW43439) — pico-node's exact target.
- **NinoTNC link**: the Pico's hardware UART wired directly to the NinoTNC's UART
  pins, **bypassing the NinoTNC's onboard USB-serial bridge** — exactly the path
  HW-BRINGUP Gate 6 plans (no USB host on the Pico; the RP2040's single USB
  controller is device-only here, used for power/probe).
- **OLED**: optional user-installed SSD1306 128×32/64 over I2C.
- **SD card**: SPI (pico-node does not use it — left free).
- **Passthrough switch**: a GPIO that, held at boot, bridges the NinoTNC UART
  straight to USB so the NinoTNC's own firmware can be updated.
- **Power**: via the NinoTNC's USB-B.

## Pin map (NinoBLE Rev5 → pico-node)

| Function | RP2040 GPIO | Peripheral | pico-node use |
|---|---|---|---|
| NinoTNC UART **TX** | **GP20** | UART1 TX | `transports::kiss_serial` (57600 8N1, KISS) |
| NinoTNC UART **RX** | **GP21** | UART1 RX | `transports::kiss_serial` |
| OLED **SDA** | **GP4** | I2C0 SDA | `oled` status display (SSD1306 @ 0x3C) |
| OLED **SCL** | **GP5** | I2C0 SCL | `oled` |
| Passthrough switch | **GP6** | GPIO in | (optional) boot-time NinoTNC-flash passthrough |
| SD card SCK / MOSI / MISO / CS | GP10 / GP11 / GP12 / GP9 | SPI1 | unused — kept free |
| Onboard LED | CYW43 WL_GPIO 0 | — | "radio alive" (already used) |
| WiFi (CYW43 PIO-SPI) | GP23/24/25/29 + DMA | PIO0 | already used — no conflict |

No conflicts: the CYW43 PIO-SPI pins (23/24/25/29) and the above are disjoint.

## What changes in pico-node

1. **`kiss_serial` → UART1 on GP20/GP21** (was the planning default UART0 GP0/GP1).
   This is the pin-compat change; the KISS codec + NinoTNC mode catalog are
   already host-tested.
2. **`oled` status module** — SSD1306 over I2C0 GP4/GP5, mirroring the NinoBLE
   firmware's proven init sequence (`oled.c`), showing node status (callsign +
   mode, IP/AP, neighbour + route counts). Optional (the OLED is user-installed);
   built in but a no-op if no panel responds at 0x3C.
3. GP6 passthrough is a documented option (not yet wired) — held low at boot it
   would put the UART into transparent bridge mode for NinoTNC firmware updates.

## Verification status

The pin map + `kiss_serial` UART selection are **code-complete and build-clean**
but **not yet hardware-verified** — the NinoBLE board + a NinoTNC + a radio are
not attached to the current dev rig (the bare Pico W on the bench has no NinoTNC
or OLED). Closing HW-BRINGUP Gate 6 (KISS-over-serial to a real NinoTNC) and
lighting the OLED both wait on wiring Tom's board to a probe-equipped machine.
The OLED init mirrors the NinoBLE firmware's known-good sequence for this exact
panel, so first-light risk is low.
