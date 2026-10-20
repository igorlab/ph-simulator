# Measuring the input leakage of a pH board

The calibrator measures the input leakage current of a pH-electrode board: the
electrometer amplifier's own input current plus surface currents (dirt, moisture, flux
residue). A good board leaks picoamps. Tens of nanoamps mean a board that will not see a
real electrode at all, because a glass electrode has 100 MΩ - 1 GΩ of internal resistance
and the leakage drops all of its signal across it.

## The idea

The simulator is a voltage source `E` followed by a high series resistor R (50 MΩ in the
reference build) into the board input. The calibrator measures two things:

- **E**: the source voltage, taken *before* the resistor. Channel A (ADS1115 AIN0-AIN1
  differential, in mV).
- **S**: the output of the board under test (0..5 V, AIN2 through a ÷2 divider).

An ideal electrometer draws no current, so there is no drop across R and the board input
equals E. A real leakage `I` flows through R and the input becomes `Vin = E - I·R`.
`Vin` cannot be measured directly: touching the node after R would make the ADS input
itself the leakage (its input resistance is in the MΩ range). But `Vin` shows up in S:
the board turns its input into its output along a straight line, and that line can be
calibrated.

## What you need

- A button that shorts R (pressed) or leaves it in circuit (released).
- The value of R. Enter it on the page in **Resistor R, MΩ**; it is kept in NVS.
- A two-point calibration (step 1).

## Step 1. Calibration (button PRESSED, R shorted)

With R shorted the board input is exactly E, so these are clean reference points.

1. Set a voltage E with the trimmer (watch A at the top of the page) and press
   **Take point 1**.
2. Set a clearly different E - the polarity switch gives `+E` and `-E`, the farther apart
   the better - and press **Take point 2**.
3. The calibrator fits the board's transfer line:

```
gainS = (S2 - S1) / (E2 - E1)     // V per mV
zeroS = S1 - gainS · E1           // V at E = 0
```

The status turns **ready** and the *Calibration* chart shows a green line through two
white points. NVS namespace `leak` keeps h1, h2, E1, S1, E2, S2, R and the limit. Two
points with the same E are useless (division by ~0); the status then stays **not ready**.

## Step 2. Leakage (button RELEASED)

Now the signal goes through R. Every cycle the calibrator works out

```
Vin   = (S - zeroS) / gainS       // the board input recovered from its output, mV
Ileak = (Vin - E) / R             // the leakage current
```

- **Leakage current (nA)**: the large amber tile in *Block 1*.
- **Over the limit**: how many times the leakage exceeds the limit you set (5 nA by
  default).
- **Leakage over time**: the chart below. The history lives in the browser (1 point/s,
  ~14 h) and survives a page reload; **Download CSV** saves it.

## Example

`E = -93 mV`, `S = 1.60 V`, and the calibration gives `Vin = +66 mV`. Then
`I = (66 - (-93)) mV / 10 MΩ = 159 mV / 10 MΩ ≈ 16 nA` (that measurement used a 10 MΩ
resistor). A good board reads at least a hundred times less.

## What it is good for

- **Absolute leakage**: read the nA with the button released.
- **Leakage over time**: watch the curve while a washed board dries - it should fall.
  A rise from moisture or a finger shows up just as clearly.
- **Comparing boards** at the same R and E.

## Resolution

With a 50 MΩ resistor and a board gain of about 6 V/V, one ADS1115 count on S (125 µV
×2 for the divider) is about 40 µV at the board input, i.e. under 1 pA of leakage. That
is the arithmetic limit; in practice mains hum on the high-impedance node, the board's
own noise and drift set the floor. Measure it once with a board you trust. A larger R
(1 GΩ) gives more sensitivity, but then the wiring, the button and your fingers start to
leak as much as a bad board: keep that node in air or on PTFE, inside a metal box.

## Pitfalls

- **Never connect the ADS after the resistor.** Its input resistance would itself become
  the leakage. E is measured before R and `Vin` is recovered through S - that is the
  whole point of the method.
- **R must match the hardware.** The calculated current is inversely proportional to R;
  a wrong R gives wrong nA.
- **Calibrate with the button PRESSED.** With it released the points already contain the
  leakage and the line is wrong.
- **0.0 nA with the button pressed is normal**: the present point lies on the line
  (Vin = E). The leakage appears only once the button is released.
- **The sign of E matters.** The calculation uses the signed E (with `A_SIGN`), so the
  direction of the current is kept. Check the sign once, see the README.

## Where it is in the code (`src/main.cpp`)

- `computeLeakCal()`: `gainS` and `zeroS` from the two points, sets `calValid`.
- `loadLeak()` / `saveLeak()`: NVS namespace `leak`.
- `loop()`: after `readA()` / `readS()` it works out `g_Vin_mV` and `g_Ileak_nA`.
- `processWebRequests()`: handles `req_capP1` / `req_capP2` / `req_calRst` / `req_setR`
  (I2C and NVS stay in `loop`).
- `/data` (JSON) serves `il` (nA), `vin` (mV), `gain` (V/V = `gainS·1000`), `zero`,
  `valid`, `e1, s1, e2, s2, h1, h2, r`. The page draws both charts from it.
