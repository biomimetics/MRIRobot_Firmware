# PWM Timer Notes — TIM3 & TIM4

## Overview

TIM3 and TIM4 generate the PWM signals for all 7 USMs (ultrasonic motors). Both timers share the same configuration and the same `ARR_PERIOD` value.

| Timer | Channels | USMs |
|-------|----------|------|
| TIM3  | CH1–CH4  | USM0–USM3 (PB4, PB5, PC8, PC9) |
| TIM4  | CH1–CH3  | USM4–USM6 (PB6, PB7, PB8) |

---

## Clock Chain

```
HSI (16 MHz)
  └─ PLL: × (PLLN=336 / PLLM=16) / PLLP=4  →  SYSCLK = 84 MHz
       └─ AHB prescaler = 1                 →  HCLK   = 84 MHz
            └─ APB1 prescaler = 2           →  PCLK1  = 42 MHz
                 └─ APB1 prescaler ≠ 1,
                    so timer clock × 2      →  TIM3/TIM4 clock = 84 MHz
```

TIM3 and TIM4 sit on the APB1 bus. Per STM32F4 reference manual, when the APB
prescaler is not 1, the timer input clock is 2× the APB peripheral clock.

**Where to change the clock:** `vel_control_mri_arm/STM_sdk/Core/Src/main.c` →
`SystemClock_Config()`. The relevant fields are `PLLM`, `PLLN`, `PLLP`, and
`APB1CLKDivider`.

---

## PWM Frequency Formula

```
f_PWM = f_timer / ((PSC + 1) × (ARR + 1))
```

| Parameter | HAL field | Current value | Effect |
|-----------|-----------|---------------|--------|
| Timer clock | (derived above) | 84 MHz | base clock |
| Prescaler (PSC) | `htim3.Init.Prescaler` | 0 | divide clock by PSC+1 |
| Auto-reload (ARR) | `htim3.Init.Period` = `ARR_PERIOD` | 2000 | sets period length in ticks |

Current settings (`common.h` ARR_PERIOD=2000, `MX_TIM3_Init` PSC=0, derived timer clock=84 MHz):
```
f_PWM = 84,000,000 / ((0+1) × (2000+1)) = 84,000,000 / 2001 ≈ 41,979 Hz (~42 kHz)
PWM resolution: 2001 counts  (duty cycle step ≈ 0.050%)
```

---

## ARR_PERIOD Trade-off Table

TIM3 and TIM4 are 16-bit timers, so ARR is bounded to [0, 65535].

| ARR_PERIOD | f_PWM    | Resolution (steps) | Duty cycle step |
|------------|----------|--------------------|-----------------|
| 1000       | ~83.9 kHz | 1001              | 0.100%          |
| 2000 *(current)* | ~42.0 kHz | 2001        | 0.050%          |
| 4000       | ~21.0 kHz | 4001              | 0.025%          |
| 8399       | ~10.0 kHz | 8400              | 0.012%          |
| 16799      | ~5.0 kHz  | 16800             | 0.006%          |
| 65535      | ~1.28 kHz | 65536             | 0.0015%         |

Higher ARR → finer duty-cycle resolution but lower PWM carrier frequency.
Lower PWM frequency means the analog filter cutoff can be proportionally lower,
making it easier to attenuate the carrier while preserving the DC/low-frequency
command signal.

---

## Where to Set Each Parameter

| What | File | Location |
|------|------|----------|
| `ARR_PERIOD` (ARR value) | `vel_control_mri_arm/src_c/common.h` | Line 57 |
| `Prescaler` (PSC) | `vel_control_mri_arm/src/lib/Drivers/USM.lf` | `MX_TIM3_Init()` line 79, `MX_TIM4_Init()` line 113 |
| PWM mode, polarity, fast mode | `vel_control_mri_arm/src/lib/Drivers/USM.lf` | `sConfigOC` struct in both Init functions |
| CCR (duty cycle at runtime) | `vel_control_mri_arm/src/lib/Drivers/USM.lf` | `set_motor_pwm_registers()` via `TIM3->CCR1` etc. |
| Max/min duty cycle limits | `vel_control_mri_arm/src_c/common.h` | `USM_MAX_DUTY_CYCLE`, `USM_MIN_DUTY_CYCLE` |
| System PLL / APB1 clock | `vel_control_mri_arm/STM_sdk/Core/Src/main.c` | `SystemClock_Config()` |

---

## Notes

- Both TIM3 and TIM4 use `ARR_PERIOD`, so changing it affects all 7 USMs simultaneously.
- The CCR (capture/compare register) for each channel is set at runtime as:
  `CCR = ARR_PERIOD × duty_cycle` (see `convert_duty_cycle_to_pwm_ccr()` in USM.lf).
  If you increase `ARR_PERIOD`, the CCR values scale up automatically with no
  other code changes required.
- The existing comment on line 57 of common.h notes sawtooth-like artifacts at
  ARR=2000. Increasing ARR_PERIOD will reduce those artifacts because each duty
  cycle step is a smaller fraction of the period.
- PSC is currently 0 (no prescaling). Adding prescaling is another way to lower
  f_PWM without changing ARR, but it reduces the absolute timer resolution.

---

## RC Low-Pass Filter Sizing

The goal is to convert the PWM output into a smooth analog voltage suitable as a
motor controller speed reference. A first-order RC filter is a simple starting
point, but the component values must be sized relative to the PWM frequency.

### Current filter (R = 4 kΩ, C = 0.01 µF)

```
f_c = 1 / (2π × R × C)
    = 1 / (2π × 4,000 × 0.00000001)
    ≈ 3,979 Hz (~3.98 kHz)

Ratio f_PWM / f_c = 41,979 / 3,979 ≈ 10.5
```

Worst-case peak-to-peak ripple estimate (valid when RC >> T_PWM; used here as a
conservative bound — the ratio RC/T_PWM ≈ 1.7× means this filter is not yet in
the well-integrated regime):

```
ΔV_pp ≈ V_supply / (4 × f_PWM × R × C)    [worst case at D = 50%]
       = V_supply / (4 × 41,979 × 4,000 × 0.00000001)
       = V_supply / 6.72
       ≈ 14.9% of V_supply

At 3.3 V supply: ~490 mV peak-to-peak
At 5.0 V supply: ~745 mV peak-to-peak
```

**This is significantly undersized for a precision analog reference.**  The RC
time constant (40 µs) is only ~1.7× the PWM period (23.8 µs), so the capacitor
barely integrates before the next PWM edge.

---

### Design rule of thumb

For a smooth analog reference, target a ripple below 1% of the supply rail.
Rearranging the ripple formula:

```
R × C  ≥  V_supply / (4 × f_PWM × ΔV_target)
       =  1 / (4 × f_PWM × (ΔV_target / V_supply))

For 1.0% ripple:  RC ≥ 1 / (4 × f_PWM × 0.010) = 596 µs
For 0.5% ripple:  RC ≥ 1 / (4 × f_PWM × 0.005) = 1.19 ms
For 0.1% ripple:  RC ≥ 1 / (4 × f_PWM × 0.001) = 5.96 ms
```

*(f_PWM = 41,979 Hz at ARR_PERIOD=2000, PSC=0)*

---

### Component options with R = 4 kΩ

| Target ripple | Required RC | C (exact) | C (standard) | f_c   | Settling time (5τ) |
|---------------|-------------|-----------|--------------|-------|--------------------|
| ~15% (current) | 40 µs      | 10 nF     | **0.01 µF**  | 3,979 Hz | 0.20 ms         |
| 1.0%          | 596 µs      | 149 nF    | **150 nF**   | 265 Hz   | 2.98 ms          |
| 0.5%          | 1.19 ms     | 298 nF    | **330 nF**   | 121 Hz   | 5.97 ms          |
| 0.1%          | 5.96 ms     | 1,490 nF  | **1.5 µF**   | 27 Hz    | 29.8 ms          |

**Recommended starting point: 150 nF** — brings ripple under 1% with a settling
time under 3 ms. If the downstream controller samples slowly or the command
changes infrequently, 330 nF is a better choice.

---

### Trade-offs and other considerations

**Settling time vs. ripple:** Lower ripple requires a larger RC and therefore a
longer settling time (5τ). If the motor controller updates its reference faster
than ~3 ms, the filter output will lag. Size the filter to the slower of the two
constraints (ripple budget or bandwidth budget).

**Downstream loading:** The motor controller's reference input impedance (Z_in)
forms a voltage divider with R. For < 1% DC error: Z_in > 100 × R = 400 kΩ.
Most op-amp reference inputs meet this easily, but verify the motor controller
datasheet. If Z_in is too low, a unity-gain buffer (op-amp follower) between the
RC output and the motor controller input eliminates this error.

---

### Option A — Add a capacitor to ground (parallel with existing C)

The existing R (4 kΩ) is on the PCB and cannot easily be changed. Adding a
capacitor to ground at the filter output node places it **in parallel** with the
existing 10 nF, so the total capacitance is simply C_total = 10 nF + C_add.
Because 10 nF is negligible compared to any useful addition, C_add ≈ C_total.

This keeps the single-pole (-20 dB/decade) roll-off shape; only f_c changes.

**At ARR_PERIOD = 2000 (f_PWM ≈ 41,979 Hz), R = 4 kΩ:**

| Target ripple | RC required | C_add (exact) | C_add (standard) | f_c    | Settling (5τ) |
|---------------|-------------|---------------|------------------|--------|---------------|
| 1.0%          | 596 µs      | 149 nF        | **150 nF**       | 265 Hz | 2.98 ms       |
| 0.1%          | 5.96 ms     | 1,490 nF      | **1.5 µF**       | 27 Hz  | 29.8 ms       |

**At ARR_PERIOD = 4000 (f_PWM ≈ 20,990 Hz), R = 4 kΩ:**

```
f_PWM = 84,000,000 / ((0+1) × (4000+1)) ≈ 20,990 Hz
RC required for 1.0% ripple  = 1 / (4 × 20,990 × 0.010) ≈ 1.19 ms
RC required for 0.1% ripple  = 1 / (4 × 20,990 × 0.001) ≈ 11.9 ms
```

| Target ripple | RC required | C_add (exact) | C_add (standard) | f_c    | Settling (5τ) |
|---------------|-------------|---------------|------------------|--------|---------------|
| 1.0%          | 1.19 ms     | 298 nF        | **330 nF**       | 121 Hz | 5.97 ms       |
| 0.1%          | 11.9 ms     | 2,975 nF      | **3.3 µF**       | 12 Hz  | 59.7 ms       |

Note: lowering f_PWM (raising ARR_PERIOD) roughly doubles the required C for the
same ripple target. At 0.1% with ARR=4000, the 59 ms settling time is likely too
slow for responsive motor control — 1% is a more practical target at that PWM
frequency.

---

### Option B — Two-stage RC filter in series

Adding a second R2-C2 stage after the existing filter gives -40 dB/decade
roll-off. For the same ripple, the cutoff frequency of each stage can be ~10×
higher than an equivalent single-pole filter, yielding ~10× faster settling time.

Each stage contributes independently, so the combined attenuation at f_PWM is:

```
H_total(f) = H_stage1(f) × H_stage2(f)
           ≈ (f_c1/f_PWM) × (f_c2/f_PWM)   [when f_PWM >> f_c]
```

A practical choice is to make both stages identical (f_c1 = f_c2 = f_c), which
simplifies the design. The required f_c per stage for a ripple target ΔV/V is:

```
f_c = f_PWM × sqrt(ΔV / V_supply)   [two identical stages]
```

**At ARR_PERIOD = 4000 (f_PWM ≈ 20,990 Hz):**

```
For 1.0% ripple:  f_c = 20,990 × sqrt(0.010) ≈ 2,099 Hz
For 0.1% ripple:  f_c = 20,990 × sqrt(0.001) ≈  664 Hz
```

With R2 = 4 kΩ (matching the existing stage), C2 = 1 / (2π × f_c × R2):

| Target ripple | f_c per stage | C2 (exact) | C2 (standard) | Settling (5τ per stage) |
|---------------|---------------|------------|----------------|--------------------------|
| 1.0%          | ~2,099 Hz     | 19.0 nF    | **22 nF**      | ~0.44 ms                |
| 0.1%          | ~664 Hz       | 60.0 nF    | **68 nF**      | ~1.39 ms                |

The two-stage approach requires routing the signal through an additional R2 in
series before connecting C2 to ground — one extra resistor and capacitor per
channel, but settling times are 5–40× shorter than Option A for the same ripple.
