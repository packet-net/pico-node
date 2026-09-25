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
| NinoTNC UART **TX** | **GP20** | UART1 TX | `ports::ninotnc` (57600 8N1, KISS) |
| NinoTNC UART **RX** | **GP21** | UART1 RX | `ports::ninotnc` |
| OLED **SDA** | **GP4** | I2C0 SDA | `oled` status display (SSD1306 @ 0x3C) |
| OLED **SCL** | **GP5** | I2C0 SCL | `oled` |
| Passthrough switch | **GP6** | GPIO in | (optional) boot-time NinoTNC-flash passthrough |
| SD card SCK / MOSI / MISO / CS | GP10 / GP11 / GP12 / GP9 | SPI1 | unused — kept free |
| Onboard LED | CYW43 WL_GPIO 0 | — | "radio alive" (already used) |
| WiFi (CYW43 PIO-SPI) | GP23/24/25/29 + DMA | PIO0 | already used — no conflict |

No conflicts: the CYW43 PIO-SPI pins (23/24/25/29) and the above are disjoint.

## What changes in pico-node

1. **The NinoTNC port (`ports::ninotnc`, formerly `kiss_serial`) on UART1 GP20/GP21** (was the planning default UART0 GP0/GP1).
   This is the pin-compat change; the KISS codec + NinoTNC mode catalog are
   already host-tested.
2. **`oled` status module** — SSD1306 over I2C0 GP4/GP5, mirroring the NinoBLE
   firmware's proven init sequence (`oled.c`), showing node status (callsign +
   mode, IP/AP, neighbour + route counts). Optional (the OLED is user-installed);
   built in but a no-op if no panel responds at 0x3C.
3. GP6 passthrough is a documented option (not yet wired) — held low at boot it
   would put the UART into transparent bridge mode for NinoTNC firmware updates.

## Verification status

**The NinoTNC link is verified on air (2026-09-25):** a Pico W on the NinoTNC's
J5 (USB chip out of circuit) set the TNC's mode by SETHW and read it back,
updated the TNC from firmware 3.39 to 3.44 through its bootloader, and carried
a connected-mode session with LinBPQ (M0LTE via QtTermTCP): UA, the console
banner, and multi-frame replies. See docs/PLAN.md §11 for the runs.

The OLED path is still unverified on this board; its init mirrors the NinoBLE
firmware's known-good sequence for this exact panel.

## Setting up the NinoTNC from the web page

Browse to the node and follow **NinoTNC setup and monitor** (or go straight to `/tnc`). The page needs the node's callsign set; until then the serial link does not start.

**Firmware 3.44 / 4.44 or later is required.** At start the node asks the TNC for its report (GETALL, repeated every 10 s until it answers) and uses the TNC only once it has reported supported firmware: then it sends the saved settings and opens the radio port. An older TNC gets no settings and no traffic; the page shows a banner and only the firmware update.

The radio port is a full node port: connected-mode sessions (SABM/UA, the node console, `C` onward to any port), XID answered with DM so v2.2 callers fall back to SABM, NET/ROM (NODES heard and originated, L4 circuits, interlinks), with one routing table for the node.

- **Operating mode.** Set all four MODE DIP switches on the NinoTNC to 1 (on), pick the mode and press **Set mode**. The node sends KISS SETHW, waits 1.5 s, asks the TNC which mode it is running, and retries up to three times, so the line under the button ends with either "confirmed by the TNC" or a plain reason (for example the DIP switch position it read). The node remembers the mode and sends it at every start without writing the TNC's own memory; tick **Also store it in the TNC's own memory** only if the TNC should keep it when used without the node.
- **KISS parameters.** TXDELAY, PERSIST, SLOTTIME, TXTAIL and duplex. TXDELAY is only used when the TNC's TX DELAY knob is fully anticlockwise (zero); otherwise the knob sets it. Values are saved on the node and sent at every start. The console keys `TNC_MODE`, `TXDELAY`, `PERSIST`, `SLOTTIME`, `TXTAIL` and `DUPLEX` set the same things (`SET TXDELAY none` goes back to the TNC's own value).
- **Test transmission.** Sends one UI frame from the node's callsign. It appears as a TX line in the monitor, and the TNC's PTT light should flash.
- **Monitor.** Also shown on the node's front page (the Radio section). Frames heard (RX), frames sent (TX) and TNC events such as mode changes and status reports, updated every second. **Ask the TNC for its status** requests a fresh report and shows the answer under the button (firmware version, DIP position, running mode), or says there was none.
- **TNC firmware.** Updates the NinoTNC over the J5 serial link, so the USB chip is not needed. Download `N9600A-v3-44.hex` (firmware 3.x) or `N9600A-v4-44.hex` (4.x) from [flashtnc](https://github.com/ninocarrillo/flashtnc), choose it and press **Upload to the node**. The node checks every line (record checksums, the end-of-file record, and the chip fingerprint that tells a 3.x file from a 4.x one) while storing it in its APPDATA flash region (about 374 KB). Then **Update the TNC** runs the flashtnc procedure from the node: quiet the line, enter the TNC's bootloader, check the bootloader is for the same chip as the file, and send the file a line at a time. It takes a few minutes with the TNC off the air. A wrong-chip file is refused before anything is written. If an update is interrupted part way, the TNC waits in its bootloader (LEDs dark); pressing **Update the TNC** again finishes it. Afterwards the node asks the updated TNC for its report and then sends its saved mode and KISS parameters.

  The upload is unauthenticated, like the node firmware upload: anyone on the node's network can use it.
