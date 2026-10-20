# Case study: a pH board that ignored a working electrode

The board that prompted this calibrator. A real bench log, with real numbers: what was
changed, what it showed, and what it ruled out. **The root cause on the old board was
never found** - the board was replaced by a re-designed one, which works. The numbers are
still useful: they show what a leaky input looks like from the outside, and how far a
simple leakage meter gets you.

## The setup

- **Calibrator**: ESP32-C3 + ADS1115 + OLED. Measures E (the simulator) and S (the board
  output) and works out the leakage.
- **Simulator** at the time: LR44, polarity switch, a 10k + 10k divider, and a series
  resistor (first 10 MΩ, then 20 MΩ) shorted by a switch.
  Switch **closed** = R shorted (calibration). Switch **open** = R in circuit
  (measurement).
- **Method**: calibration (closed) gives the line `S = zeroS + gainS·E`; measurement (open)
  gives `Vin = (S - zeroS)/gainS` and `leak = (Vin - E)/R`.

## The board

- Input: BNC centre → pin 5 (+) of U4 = **LMC6042AIM** (dual: U4.1 input buffer + U4.2).
  Pin 5 is lifted in the air, no solder pad, nothing touching the laminate.
- U4.1: non-inverting, R3 = 255k feedback, **R8 = 39k** gain resistor to the inverting
  input, RA1 = 10k offset. Gain 1 + R3/R8 ≈ 7.5.
- Signal path: U4.1 → U4.2 → R4 100k → U3 (LMC6042) → R2 → U2 (HCNR201 linear opto) →
  U1 (LMC6042) → output S.
- ±5 V from an A0505S-1W isolated DC-DC converter.

## Symptoms

1. The calibrator shows a "leakage": the input is shifted by **90-205 mV**, about
   **8-13 nA**, i.e. an error of 1.5-3.5 pH.
2. A **good electrode** (it works on a commercial pH meter) **does not move** the output of
   this board at all.

The board used to work with an electrode, without a guard ring.

## What was changed and what it showed

| Change | Result |
|---|---|
| Input op-amp replaced (LMC6042) ×2 | leakage exactly the same |
| **TL062** (JFET, pA input current) | leakage stays (166 mV) |
| New BNC | no change |
| Flux cleaned off (visibly clean) | no change |
| **R8 removed** completely | leakage unchanged; gain → 1 |
| R8 put back | gain back to ~-6.6 |
| **Heating around R8 / U4.1** | leakage swings a lot (dips to -21 nA, transient) |
| **R 10 → 20 MΩ** | shift **166 → 205 mV** (×1.23, less than proportional) |
| Multimeter (DC V, ~10 MΩ) on the BNC | jumps around |
| The same electrode on a pH meter | **responds** (electrode is fine) |

## A direct test, without the simulator

- Board input **shorted** (BNC centre to shield) → S₀. Then the input to the shield
  **through 10 MΩ** → S₁. The difference shows a leakage on the board itself; the
  simulator, its battery and channel A take no part.
- `ΔVin = (S₁ - S₀)/gainS`, `Ileak = ΔVin / 10 MΩ`.
- Measured (TL062 + R8; multimeter on the U4.1 output, pin 7, against the BNC body):

  | Input | V(pin 7) | Vin (÷6.58) | Current |
  |---|---|---|---|
  | shorted | 3.5 mV | ~0.5 mV (op-amp offset) | - |
  | through 10 MΩ | 0.824 V | +125 mV | 12.5 nA |
  | through 20 MΩ | 1.62 V | +246 mV | 12.3 nA |

  The effect enters through the U4.1 input (pin 7 moves). **The current is the same at
  10 and 20 MΩ and the node goes positive** - that is a **current source of about 12 nA**,
  not a resistance to ground (a resistance to ground gives no current with the input
  shorted).
- U3 output (pin 7) against the BNC body: 1.75 V shorted, 2.4 V through 10 MΩ. Both in
  the linear range, not at a rail, following U4.1. LED current of the HCNR201
  ≈ (5.5 - 1.6 - V)/255 Ω ≈ 8.4 mA and 5.9 mA - normal (limit 25 mA). U3 is fine.

## Gain over time

| State | Gain |
|---|---|
| Original (LMC6042, R8 = 39k) | -13.9 V/V |
| R8 "reworked" / open | -1.0 |
| R8 removed | -1.0 |
| TL062 + R8 back | -6.58 |

## Ruled out

- **The input op-amp (silicon)**: three different parts (2× LMC6042 CMOS + TL062 JFET)
  give the same leakage. A pA-class TL062 cannot produce ~8 nA.
- **R8**: removed completely - leakage unchanged (R8 only sets the gain).
- **A trace or pad at pin 5**: there is none - the input is pin 5 in the air plus the BNC.
- **A missing guard**: the board worked without one before.
- **Visible flux or dirt**: washed off - no change.

## One calculation that ties both symptoms together

The board responds to the **20 MΩ** simulator (with a large shift) but **not** to a
**100 MΩ - 1 GΩ** electrode. That fits a **low resistance from the input node to ground**,
`Rin`, which divides the source by impedance:

From the R change: `shift = E·R/(R+Rin)`; 166 mV at 10 MΩ and 205 mV at 20 MΩ give
**Rin ≈ 6 MΩ** to ground (E ≈ 270 mV).

- Electrode 200 MΩ into 6 MΩ → ~3 % gets through → **no response** ✓
- Simulator 20 MΩ into 6 MΩ → ~23 % → **responds, with a large shift** ✓

**One cause - about 6 MΩ from the input node to ground - explains both symptoms.**
It should be far above a gigaohm.

Note that the direct test above points the other way (a constant ~12 nA, not a
resistance). The two readings were never reconciled, which is part of why the cause stays
open.

## What was left

The input node is just three things: the BNC (replaced, no short), the op-amp pins
(pin 5 to pin 4 / V- or pin 6 - unlikely, it survived two SOIC parts and a TL062), and
whatever couples to them through the air and the laminate.

## Not measured (gaps)

- **Input node to ground with an ohmmeter.** The log has "30 mΩ" here, which cannot be
  right - a short would kill the signal completely. Most likely a misreading; worth
  re-measuring on any board like this one.
- **The simulator's own resistance** from its output to ground.
- **Grounding**: E is measured against the simulator reference, the board against its
  own ground - the two were never compared directly.
- (The ±5 V rails were checked: stable, +5.5 / -5.5 V.)

## The new board (KiCad, no trimmers) - 2026-10-19

Laid out again from scratch: RA1/RA2 replaced by wire links, R3 = 200k, R5 = 56k.
Designed for S = 0.5 V at pH 4 ... 2.45 V at pH 9.38, a slope of -6.13 V/V.

| Input | S |
|---|---|
| +101 mV | 1.059 V |
| -100.2 mV | 2.32 V |

- Slope **-6.27 V/V** (+2.3 % from the design value: HCNR201 and resistor tolerances),
  S(0 mV) = **1.692 V**.
- **Shorting the 20 MΩ series resistor does not move S.** The input current is below what
  a multimeter can resolve: at 1 mV on S that is < 1/6.27 mV / 20 MΩ ≈ 8 pA. The old
  board gave 12 nA.
- The zero follows the +5 V rail (about 5.5 V, as on the old board): Vref = 0.152·V(+5V),
  so S moves ≈ 0.31 V per volt of rail, ≈ 0.04 pH per 50 mV. A two-point calibration
  removes the constant part; calibrate with the board powered the way it will be used.
- On this line (Nernst, 25 °C): pH 4.01 → 0.58 V, pH 6.86 → 1.64 V, pH 8.3 → 2.17 V. The
  linear range of the ESP32's own ADC (11 dB, up to 2.45 V) ends at pH ≈ 9.0.
