## Which file do I need?

| File | Use it for |
|---|---|
| `pico-node-app.bin` | **Upgrading a node that is already running pico-node.** In the node's web panel, under **Firmware** (not the TNC page's "TNC firmware", which is for the NinoTNC), choose this file and upload it. The node restarts on the new version; if it misbehaves it rolls back to the previous one on the next restart. |
| `pico-node-firmware.uf2` and `pico-node-blobs.uf2` | **A fresh install** on a new Pico W, or one that is not running pico-node. You need **both**, dropped one at a time (below). |
| `SHA256SUMS` | Checksums of the three files above, to check a download. |

### Fresh install

1. Hold **BOOTSEL** while plugging the Pico W into USB. It appears as a drive called `RPI-RP2`.
2. Drag `pico-node-firmware.uf2` onto it. The drive disappears when it has been written.
3. Hold **BOOTSEL** and plug in again, then drag `pico-node-blobs.uf2` onto it.

Either order works; the node does not start properly until both are on, so a Pico with only one looks dead, which is expected. They are two files because the RP2040's drag-and-drop loader cannot take one file that covers both flash regions. On Linux, `sudo picotool load <file>` for each, then `picotool reboot`, is more reliable than drag-and-drop.

On first start the node has no callsign and stays off the air. It opens a WiFi access point called `pico-setup` (passphrase `packetradio`); join it and browse to `192.168.4.1` to set the callsign and WiFi. See `docs/PROVISIONING.md`.

After a fresh install, later upgrades only ever need `pico-node-app.bin`.
