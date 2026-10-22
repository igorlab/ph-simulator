# pH electrode simulator and leakage tester

*[Українською](README.uk.md)*

A bench tool for pH-electrode boards (electrometer front ends). A battery-powered
simulator feeds a known voltage E to the board through a high series resistor, the way a
glass electrode would. An ESP32-C3 with an ADS1115 measures E and the board output S,
works out the board's gain and zero from two calibration points, and from then on shows
the board's **input leakage current** - the fault that makes a board ignore a perfectly
good electrode while still reacting to a low-impedance test source. Readings go to a
128x64 OLED and to a web page (EN / UA) with live charts.

Why it exists: [a board that ignored a working electrode](docs/case-study-leaky-board.md).
How the leakage is worked out: [docs/leak-measurement.md](docs/leak-measurement.md).

![Wiring diagram](docs/wiring.png)

The same circuit as a schematic: [docs/schematic.pdf](docs/schematic.pdf) (KiCad 10 project in
[hardware/](hardware/)).

## What it shows

- **A**: the simulator voltage in mV, and the pH it stands for (`pH = 7 - A/59.16`, 25 °C).
- **S**: the board output in V, and the pH the board reports after calibration.
- **Leakage current** in nA, how many times it exceeds your limit, and a chart over time.
- The board's **gain and zero** (two-point line E → S), the **noise** on S and the
  board's **supply current**.
- **Electrode equilibrium**: how fast a real electrode settles after it is dipped
  (time to equilibrium, t63, drift).
- History in the browser (1 point/s, ~14 h), downloadable as CSV.

## Parts

| | |
|---|---|
| Controller | ESP32-C3 SuperMini |
| ADC | ADS1115 module (I2C 0x48), powered from 3.3 V |
| Display | SSD1306 128x64 OLED, I2C 0x3C |
| Simulator | LR44 cell, DPDT ON-OFF-ON switch (polarity), 6.8 kΩ, 10 kΩ trimmer |
| Series resistor | 50 MΩ (five 10 MΩ SMD resistors in series), shorted by a push button for calibration |
| Board connector | BNC female; JST 3-pin (S, +5 V, GND) |
| S divider | R3 = R4 = 10 kΩ (the ADS gets S/2) |
| Current shunt | Rsh (R5) = 10 Ω in the board's ground |

Wire by the pin names printed on the modules. Never connect the BNC centre (the
high-impedance node) to the ADS - its input would itself become the leakage.

Connect the JST to the board under test by function - +5 V to its supply, GND to its
ground, S to its output - and check the pin order at both ends before you power it up.
The JST here has +5 V in the middle; the pH board of this project has its ground there
(P2: 1 VCC, 2 com, 3 S), so the cable between them is crossed.

## Build and flash

[PlatformIO](https://platformio.org/) Core 6.2.0 or newer (pioarduino needs it). In the
folder with `platformio.ini`:

```bash
pio run -t upload -t monitor
```

Every version is pinned in [`platformio.ini`](platformio.ini) - the platform
(pioarduino 55.03.312-1 = Arduino core 3.3.12, ESP-IDF 5.5.5, GCC 14.2) and each library
(Adafruit ADS1X15 2.6.2, Adafruit BusIO 1.17.4, U8g2 2.36.18, AsyncTCP 3.5.0,
ESPAsyncWebServer 3.12.1). The first build downloads them. Do not loosen the pins, or
the project will stop building in a year.

### Port

`platformio.ini` names no port: with one board plugged in, PlatformIO finds it on its own.
With several (another ESP on the same USB hub, say), tell it which one. Either once:

```bash
pio run -t upload --upload-port /dev/cu.usbmodem1101
```

or for good, in `platformio_local.ini` next to `platformio.ini` (ignored by git, picked up
by `extra_configs`):

```ini
[env:esp32-c3-devkitm-1]
upload_port  = /dev/cu.usbmodem1101
monitor_port = /dev/cu.usbmodem1101
```

Find the name with `pio device list`. The ESP32-C3 SuperMini uses native USB, so it shows
up as `/dev/cu.usbmodem…` on macOS, `/dev/ttyACM0` on Linux and `COMx` on Windows.

## Wi-Fi

No network is compiled in. On first start the calibrator cannot join anything, so it
brings up its own access point:

| | |
|---|---|
| Network | `pH-Calibrator` |
| Password | `calib1234` |
| Page | `http://192.168.4.1/` (a phone usually opens it by itself) |

Open the **Wi-Fi** block at the bottom of the page, press **Find networks**, pick yours,
type the password and press **Save and restart**. The calibrator joins it and from then on
answers at `http://calibrator.local/` (or the IP it shows on the OLED for a few seconds
after start). Only 2.4 GHz networks work.

If the saved network is out of reach, the calibrator falls back to the same access point
after 20 s; **Forget the network** sends it there straight away. A network that drops
after the calibrator has joined it is reconnected automatically.

For your own builds you can also put a default network in `src/wifi_secret.h` (ignored by
git). It is used only until a network is saved on the page:

```c
#pragma once
#define WIFI_SSID "my-network"
#define WIFI_PASS "my-password"
```

The page has no password: use it on a network you trust.

## Sign of channel A

Channel A (the simulator voltage, ADS1115 AIN0-AIN1) has a sign that depends on which way
round the two wires are. Check it once after assembly:

1. Press and hold the button (the series resistor is shorted).
2. Put a multimeter on DC mV across the BNC: **red to the centre, black to the shield**.
3. Compare its sign with **A** on the OLED or the page.

Same sign - nothing to do. Opposite sign - in [`src/main.cpp`](src/main.cpp) change
`A_SIGN` (`-1.0f` to `1.0f` or back) and flash again, or swap the AIN0 and AIN1 wires. The
size should match the meter to within a few tenths of a mV; **Autozero A** on the page
removes a constant offset (with the trimmer at its zero end).

The sign matters: pH on the page is `7 - A/59.16` (positive mV = acid), and the leakage
is calculated from the difference between A and the board's input, so a wrong sign gives
a wrong pH and a meaningless leakage figure.

## Measuring a board

1. Button **pressed**: set some E with the trimmer, **Take point 1**. Set a clearly
   different E (flip the polarity switch), **Take point 2**. The line is **ready**.
2. Enter your series resistor in **Resistor R, MΩ** (50 by default).
3. Button **released**: read the leakage. Watch the chart while a washed board dries.

Details, formulas and pitfalls: [docs/leak-measurement.md](docs/leak-measurement.md).

The page's «Download CSV» keeps only time, pH and S. To keep every reading - E, S, the
board input, the leakage and the calibration line - log what the page itself polls:
`python3 tools/calibrator_log.py -o run.csv` (type a note and Enter to mark a phase).

## Serial

USB CDC, 115200 baud. One CSV line `millis,A_mV,S_V,pH` per measurement cycle (about
0.4 s: each channel averages 25 samples). Commands:

| | |
|---|---|
| `z` | autozero A |
| `Z` | clear the autozero |
| `r` | print the autozero |
| `v` | wiring check: each ADS input AIN0..AIN3 against GND, in volts |

## Limits

- The pH shown for A assumes the ideal Nernst slope at 25 °C; there is no temperature
  compensation - it is a simulator, not a meter.
- Leakage resolution is set by R and noise, not by the arithmetic. With 50 MΩ one ADC
  count is under 1 pA; mains hum and drift on the high-impedance node are larger. A
  bigger R (1 GΩ) needs the node in air or on PTFE, inside a metal box.
- The board under test is powered from USB 5 V. A board whose zero follows its supply
  will shift a little if the USB voltage changes between calibration and measurement.
- Channel A needs a board. The simulator's reference - the trimmer leg, the BNC shield
  and A1 - has no divider of its own. With a board connected the readings are valid;
  without one A1 floats and A means nothing. A 10 kΩ / 10 kΩ divider from 3.3 V to GND
  with 100 nF, its midpoint on A1, would make A valid on its own; the author's unit works
  without it.

## License

Open hardware, in three parts (see [LICENSE](LICENSE)):

- hardware (wiring diagram, schematics, boards): [CERN-OHL-S v2](LICENSES/CERN-OHL-S-2.0.txt);
- firmware and scripts: [GPL-3.0-or-later](LICENSES/GPL-3.0-or-later.txt);
- documentation and images: [CC BY-SA 4.0](LICENSES/CC-BY-SA-4.0.txt).

You may build, modify, share and sell the device. A modified version you distribute must
stay under the same licences, with its sources published.
