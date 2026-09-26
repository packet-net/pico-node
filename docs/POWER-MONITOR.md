# Station power monitor

Add an INA226 current and voltage sensor and your node reports its supply voltage and current draw on air as APRS telemetry, every 10 minutes. With an APRS igate in range, the graphs show up on aprs.fi under your node's callsign.

There's nothing to switch on. The node looks for the sensor when it starts and once a minute after that, so you can fit it at any time.

## What you need

An INA226 breakout board. They're cheap and widely sold. Check the chip is marked **I226**: the similar-looking INA219 boards won't work.

## Wiring

Power the node off first.

### 1. Sensor to the Pico

Four wires. On a NinoBLE board these pins are also on the OLED header, and the display and sensor can share them.

| Sensor | Pico W |
|---|---|
| VCC | 3V3 (pin 36) |
| GND | GND (pin 38, or any GND) |
| SDA | GP4 (pin 6) |
| SCL | GP5 (pin 7) |

### 2. Sensor into the station's power

The sensor measures current through a small resistor, the **shunt**, which must sit in the **positive** feed to your station. Everything the station draws, radio included, must flow through it.

1. Disconnect the station's positive lead from the battery or power supply.
2. Battery **+** to the sensor's current input: **I+** or **IN+**.
3. The sensor's current output, **I-** or **IN-**, to the station's **+**.
4. Leave the negative side as it is: battery **−** straight to the station's **−**.

If your board has separate voltage terminals (**V+** and **V-**), connect **V+** to battery + and **V-** to battery −. Most boards without them measure the voltage at IN- by themselves.

> **Never connect I- or IN- to the negative side.** Both current terminals belong on the positive side, one either side of the break. Wiring one to negative shorts the battery through the shunt.

The wires carrying the current, battery to I+ and I- to the station, carry everything the station draws. Use wire as heavy as your station's main power lead, and check the board's terminals are rated for your radio's transmit current.

### 3. Tell the node your shunt's value

The shunt's value is printed on it: **R100** is 0.1 ohm (100 milliohms), **R010** is 10 milliohms, **R002** is 2 milliohms.

In the node's web panel, under **Configure**, set **Current shunt (milliohms)** to that number (for example `2` for an R002), then **Save & reboot**. The node assumes 100 until you change it.

The sensor reads up to 81.92 millivolts across the shunt, so the shunt value sets the most current it can measure:

| Shunt | Reads up to | Good for |
|---|---|---|
| R100 (100 milliohms) | 0.8 A | the node on its own, not a radio |
| R010 (10 milliohms) | 8 A | a small station |
| R002 (2 milliohms) | 41 A | a whole station with a 50 W radio |

## Checking it works

Power up. Within a minute the line under the web panel's heading shows the reading, for example **Power: 13.62 V, 1.25 A**.

- **Current shows negative:** I+ and I- are swapped. Swap them.
- **"no INA226" appears instead:** the node can't find the sensor. The line lists what it can see on those two pins. Nothing listed means check the four sensor-to-Pico wires, especially that SDA and SCL aren't swapped.

About a minute after start-up the node sends its first telemetry, and then one every 10 minutes.

## Settings

Both are under **Configure** on the web panel, or on the telnet console:

| Web panel | Console | Default |
|---|---|---|
| Power telemetry every (minutes, 0 = off) | `SET TELEM_INTERVAL 10` | 10 |
| Current shunt (milliohms) | `SET SHUNT_MOHM 2` | 100 |

On the console, follow with `SAVE` and `REBOOT`.

## What goes on air

Every report is a telemetry frame on the node's radio port, from the node's callsign:

- **Battery** in volts, in 0.06 V steps up to 15.3 V (a 4S LiFePO4 pack peaks at 14.6 V).
- **Current** in amps, in steps of 0.01 A to 0.1 A depending on the shunt. Current flowing back into the battery (charging) is sent as 0.

With the first report, and once an hour after that, the node also sends the labels that tell aprs.fi what the numbers mean. If a grid locator is set, it sends a position report too, so the node appears on the map. The position is the centre of the grid square, so use a 6-character locator (like `IO91lk`) for a position within a few kilometres.
