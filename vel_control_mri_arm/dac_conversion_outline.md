# DAC Conversion Outline — PWM to DAC via SPI or I2C

## Quick Summary

Two options are described in this document. Both replace the 7-channel PWM +
RC filter approach with a cleaner DAC-driven analog reference.

| | Option A — AD5628 via SPI1 | Option B — DAC7578 via I2C1 |
|-|---------------------------|------------------------------|
| Interface | SPI1 (PB3/PB4/PB5 + PB7 CS) | I2C1 (PB6/PB7) |
| Pin relocation needed | Yes — move motor 2 enable PB3 → PB6 | **None** |
| Update time (7 channels) | ~10 µs | ~665 µs at 100 kHz |
| Reference | Internal 1.25 V (0–2.5 V out) | VDD (0–3.3 V out at 3.3 V supply) |
| Prototype option | Bare IC only | Adafruit breakout available |

**Recommendation: start with Option B (DAC7578 / I2C1).**
The Adafruit breakout lets you validate the concept on the bench with four
wires (VCC, GND, SDA, SCL) and no PCB changes. No pin relocations are needed
in firmware, and the 665 µs update time is acceptable for a 10 ms control loop.
Switch to Option A if update latency becomes a bottleneck after testing.

---

## Overview

This document outlines what needs to change to replace the 7-channel PWM-based
motor speed reference (TIM3/TIM4 → RC filter → motor controller reference pin)
with a direct SPI-driven AD5628 8-channel 12-bit DAC. The AD5628 produces a
clean analog output with no PWM carrier ripple, eliminating the need for the RC
low-pass filters entirely.

The key enabling trade: removing TIM3 and TIM4 frees 7 GPIO pins that were
claimed as PWM outputs. This unblocks **SPI1** on its secondary pin set
(PB3/PB4/PB5), which is otherwise occupied by TIM3 CH1/CH2.

---

## AD5628 Overview

| Property | Value |
|----------|-------|
| Channels | 8 (DAC A–H), only 7 used |
| Resolution | 12-bit (0–4095 counts) |
| Interface | SPI, 32-bit frames, up to 50 MHz SCLK |
| Supply | 2.7 V – 5.5 V (can run at 5 V with 3.3 V logic) |
| Internal reference | 1.25 V (×2 output range = 0–2.5 V), or external |
| Output range | 0 to VDD (with VDD = supply voltage) in unbuffered mode |
| LDAC | Hardware pin or per-command; use command 0x3 to write + update immediately |

Verify the target motor controller reference input voltage range before choosing
the DAC supply voltage and reference configuration.

---

## Pin Changes

### Pins freed by removing TIM3 and TIM4

| Pin | Was | Now free for |
|-----|-----|--------------|
| PB4 | TIM3 CH1 (USM0 PWM) | SPI1 MISO (AF5) |
| PB5 | TIM3 CH2 (USM1 PWM) | SPI1 MOSI (AF5) |
| PB6 | TIM4 CH1 (USM4 PWM) | Motor 2 enable (relocated from PB3) |
| PB7 | TIM4 CH2 (USM5 PWM) | SPI1 software NSS (CS to AD5628) |
| PB8 | TIM4 CH3 (USM6 PWM) | Spare GPIO |
| PC8 | TIM3 CH3 (USM2 PWM) | Spare GPIO |
| PC9 | TIM3 CH4 (USM3 PWM) | Spare GPIO |

### One pin relocation required

PB3 is currently motor 2 enable but is also SPI1 SCK (AF5) on the secondary
pin set. Move motor 2 enable from PB3 to PB6 (freed above).

| GPIO function | Old pin | New pin |
|---------------|---------|---------|
| Motor 2 enable | PB3 | PB6 |

### Resulting SPI1 pin assignment

| SPI1 signal | Pin | AF  | Note |
|-------------|-----|-----|------|
| SCK         | PB3 | AF5 | Previously motor 2 enable — now SPI clock |
| MISO        | PB4 | AF5 | Previously TIM3 CH1 |
| MOSI        | PB5 | AF5 | Previously TIM3 CH2 |
| NSS (~CS)   | PB7 | —   | Software GPIO (active-low to AD5628 SYNC pin) |

MISO can be left unconnected if the AD5628 is write-only in this application
(no readback needed). Configure `SPI_DIRECTION_1LINE` or simply leave PB4
floating with the SPI still in 2-line mode — no harm either way.

---

## AD5628 SPI Protocol

Every transaction is exactly **32 bits** (4 bytes), MSB first, sent while
SYNC (~CS) is held low. SYNC goes high after the 32nd clock to latch the data.

```
Bit  [31:28]  Command  (4 bits)
Bit  [27:24]  Address  (4 bits)  — DAC channel 0–7
Bit  [23:12]  Data     (12 bits) — DAC value
Bit  [11:0]   Don't care         — send 0x000
```

### Relevant commands

| Command | Code | Description |
|---------|------|-------------|
| Write input register n | 0x0 | Stage value, don't update output yet |
| Write input + update DAC n | 0x3 | Write and update channel immediately — use this for single-channel updates |
| Power down/up | 0x4 | Shut down individual channels |
| Reset | 0x5 | Reset all registers to zero |
| Internal reference on | 0x7 + data=0x1 | Enable the 1.25 V internal reference |

### Channel address map (suggested)

| AD5628 channel | Address | USM |
|----------------|---------|-----|
| DAC A | 0x0 | USM0 |
| DAC B | 0x1 | USM1 |
| DAC C | 0x2 | USM2 |
| DAC D | 0x3 | USM3 |
| DAC E | 0x4 | USM4 |
| DAC F | 0x5 | USM5 |
| DAC G | 0x6 | USM6 |
| DAC H | 0x7 | unused |

### SPI mode

The AD5628 clocks data in on the rising edge of SCLK with SCLK idle low,
which corresponds to **SPI Mode 1 (CPOL=0, CPHA=1)**. Verify against the
AD5628 datasheet timing diagrams before finalising the HAL config.

---

## SPI1 HAL Init (to add to usart.c or a new spi.c)

```c
SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(void) {
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;   // CPOL=0
    hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;    // CPHA=1 → Mode 1
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; // 84/4 = 21 MHz
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi1);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (spiHandle->Instance == SPI1) {
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PB3=SCK, PB4=MISO, PB5=MOSI */
        GPIO_InitStruct.Pin       = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* PB7 = software NSS (SYNC on AD5628), idle high */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        GPIO_InitStruct.Pin       = GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = 0;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}
```

SPI1 is on APB2 (84 MHz). `BAUDRATEPRESCALER_4` gives 21 MHz, well within the
AD5628's 50 MHz limit. Drop to `BAUDRATEPRESCALER_8` (10.5 MHz) if signal
integrity is a concern on longer traces.

---

## New Helper Functions (replace PWM equivalents in USM.lf preamble)

```c
extern SPI_HandleTypeDef hspi1;

#define AD5628_CMD_WRITE_UPDATE  0x3
#define AD5628_CMD_RESET         0x5
#define AD5628_CMD_INT_REF_ON    0x7
#define AD5628_CS_PORT           GPIOB
#define AD5628_CS_PIN            GPIO_PIN_7
#define AD5628_DAC_COUNTS        4095  // 12-bit full scale

/* Send one 32-bit frame to the AD5628 */
void AD5628_write(uint8_t command, uint8_t address, uint16_t data) {
    uint32_t frame = ((uint32_t)(command & 0xF) << 28)
                   | ((uint32_t)(address & 0xF) << 24)
                   | ((uint32_t)(data    & 0xFFF) << 12);
    uint8_t buf[4] = {
        (frame >> 24) & 0xFF,
        (frame >> 16) & 0xFF,
        (frame >>  8) & 0xFF,
        (frame      ) & 0xFF
    };
    HAL_GPIO_WritePin(AD5628_CS_PORT, AD5628_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, buf, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(AD5628_CS_PORT, AD5628_CS_PIN, GPIO_PIN_SET);
}

/* Replaces convert_duty_cycle_to_pwm_ccr() */
uint16_t convert_duty_cycle_to_dac_counts(float duty_cycle) {
    return (uint16_t)(duty_cycle * (float)AD5628_DAC_COUNTS);
}

/* Replaces set_motor_pwm_registers() */
void set_motor_dac_outputs(void) {
    for (int i = 0; i < 7; i++) {
        uint16_t counts = convert_duty_cycle_to_dac_counts(usm_duty_cycle_[i]);
        AD5628_write(AD5628_CMD_WRITE_UPDATE, i, counts);
    }
}

/* Call once at startup after SPI1 init */
void AD5628_init(void) {
    AD5628_write(AD5628_CMD_RESET,    0x0, 0x000); // reset all channels
    AD5628_write(AD5628_CMD_INT_REF_ON, 0x0, 0x001); // enable internal 1.25V ref
    // Output range = 0 to 2*Vref = 0–2.5 V with internal ref.
    // If 0–VDD range is needed, use an external reference or power DAC at desired Vout_max.
    set_motor_dac_outputs(); // set all outputs to 0
}
```

---

## Changes Required Per File

### `vel_control_mri_arm/src/lib/Drivers/USM.lf`

| What to remove | What to add/change |
|----------------|--------------------|
| `extern TIM_HandleTypeDef htim3` | `extern SPI_HandleTypeDef hspi1` |
| `extern TIM_HandleTypeDef htim4` | *(remove)* |
| `MX_TIM3_Init()` function | `AD5628_write()` helper |
| `MX_TIM4_Init()` function | `convert_duty_cycle_to_dac_counts()` |
| `convert_duty_cycle_to_pwm_ccr()` | `set_motor_dac_outputs()` |
| `set_motor_pwm_registers()` (writes to TIM3/TIM4 CCR) | `AD5628_init()` |
| `HAL_TIM_PWM_Start()` calls in startup reaction | `AD5628_init()` call in startup |
| `TIM3->CCR1` etc. direct register writes | `AD5628_write()` calls |
| Motor enable for PB3 → change to PB6 in `set_motor_enable_pins()` | *(update GPIO_PIN_3 → GPIO_PIN_6)* |

### `vel_control_mri_arm/src/Main.lf` (`stm32_init`)

| Remove | Add |
|--------|-----|
| `MX_TIM3_Init()` | `MX_SPI1_Init()` |
| `MX_TIM4_Init()` | *(nothing else needed)* |
| `extern TIM_HandleTypeDef htim3/htim4` | `extern SPI_HandleTypeDef hspi1` |
| `HAL_TIM_Base_Start()` for TIM3/TIM4 | *(remove)* |

### `vel_control_mri_arm/STM_sdk/Core/Src/gpio.c` (`MX_GPIO_Init`)

| Change | Detail |
|--------|--------|
| Remove PB3 from the motor enable output group | Remove `GPIO_PIN_3` from the `GPIOB` output init block |
| Add PB6 to the motor enable output group | Add `GPIO_PIN_6` to the same block |
| PB4, PB5, PB7 no longer need GPIO init here | They will be configured by `HAL_SPI_MspInit` |

### `vel_control_mri_arm/STM_sdk/Core/Src/tim.c`

No edits strictly required — `MX_TIM3_Init` / `MX_TIM4_Init` simply won't be
called anymore. Optionally remove or `#if 0` the functions to prevent confusion.

---

## set_motor_enable_pins Update

In `USM.lf`, the motor enable function writes PB3 for motor 2. Update it to
write PB6 instead:

```c
// Before
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, set_value);  // motor 2 enable

// After
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, set_value);  // motor 2 enable (moved from PB3)
```

The same substitution applies in `set_motor_enable_pins_from_array()` and
`set_motor_enable_deadband()`.

---

## Hardware Notes

**Voltage levels:** The AD5628 SPI inputs are 3.3 V compatible when VDD = 5 V
(V_IH ≥ 2.0 V, V_IL ≤ 0.8 V per datasheet). No level shifter needed for 3.3 V
STM32 logic driving a 5 V AD5628.

**Output range:** With the internal 1.25 V reference, the output range is 0–2.5 V
(gain = 2 internally). With VDD = 5 V and an external 2.5 V reference, the range
is 0–5 V. Match this to the motor controller's reference input spec.

**LDAC pin:** Tie LDAC low to allow the command-0x3 write-and-update to take
effect immediately without a separate LDAC pulse. If simultaneous update of all
7 channels in one shot is needed later, tie LDAC high and pulse it low after
staging all channels with command 0x0.

**SYNC timing:** SYNC (CS) must stay low for the full 32 clocks. With
`HAL_SPI_Transmit` in blocking mode this is guaranteed. If DMA is used later,
ensure the CS GPIO is de-asserted in the DMA transfer-complete callback, not
immediately after the `HAL_SPI_Transmit_DMA` call returns.

**Isolation:** As noted in the PWM timer notes, if the AD5628 analog ground is
shared with a high-voltage AC line, add a digital isolator (e.g. ADUM1401) on
the SPI lines and power the DAC from an isolated DC-DC converter to break the
ground loop entirely.

---

---

# Alternative Implementation — DAC7578 via I2C1

## Overview

The DAC7578 (Texas Instruments) is an 8-channel 12-bit DAC with an I2C
interface. It is functionally equivalent to the AD5628 for this application and
may be easier to source. The I2C path is also **simpler to wire** — it uses
only 2 lines instead of 4, and requires **no pin relocations** at all compared
to the SPI1 path above.

---

## DAC7578 vs AD5628 Comparison

| Property | AD5628 (SPI) | DAC7578 (I2C) |
|----------|-------------|---------------|
| Channels | 8 | 8 |
| Resolution | 12-bit | 12-bit |
| Interface | SPI, 4 wires | I2C, 2 wires |
| Max clock | 50 MHz | 3.4 MHz (HS mode) |
| 7-channel update time | ~10 µs at 21 MHz | ~665 µs at 400 kHz |
| Pin relocation needed | Yes (PB3 motor enable → PB6) | **None** |
| Reference | Internal 1.25 V (×2 = 0–2.5 V), or external | VDD (output = 0 to VDD) |
| Supply | 2.7–5.5 V | 2.7–5.5 V |
| Package (easy to hand-solder) | SOIC-16 | TSSOP-16 (PW) or QFN-24 (RGE) |

The 665 µs update time at 400 kHz Fast mode is ~6.6% of a 10 ms control loop.
This is acceptable for most motor speed reference applications. If tighter timing
is required, the DAC7578 supports Fast mode plus (1 MHz, ~265 µs) and High-speed
mode (3.4 MHz, ~78 µs), though the STM32F446 I2C peripheral only supports up to
Fast mode (400 kHz) in standard HAL configuration — verify HS mode support before
relying on it.

---

## Pin Assignment — I2C1 on PB6/PB7

Both I2C1 pins are freed directly by removing TIM4, with no other changes:

| I2C1 signal | Pin | AF  | Was |
|-------------|-----|-----|-----|
| SCL         | PB6 | AF4 | TIM4 CH1 (USM4 PWM) |
| SDA         | PB7 | AF4 | TIM4 CH2 (USM5 PWM) |

PB3 **stays** as motor 2 enable — no relocation required.
PB4, PB5, PB8, PC8, PC9 are freed but unused; available as spare GPIOs.

---

## DAC7578 I2C Address

The 7-bit address is `1001 [AD2:AD1:AD0]`, where AD2:AD0 are set by the ADDR
pin(s).

**TSSOP-16 package (PW suffix)** — one ADDR pin (ADDR0):

| ADDR0 | 7-bit address | 8-bit write byte |
|-------|---------------|------------------|
| GND   | 0x48          | 0x90             |
| VDD   | 0x4A          | 0x94             |
| Float | 0x4C          | 0x98             |

**QFN-24 package (RGE suffix)** — two ADDR pins (ADDR0, ADDR1), supports up to
8 addresses (0x48–0x4F). Use this package if multiple DAC7578s are needed on
the same bus.

Tie ADDR0 to GND for address 0x48 in a single-device setup.

---

## DAC7578 I2C Protocol

Every write is 4 bytes: address byte, command/access (CA) byte, most significant
data byte (MSDB), least significant data byte (LSDB). Source: DAC7578 datasheet
SBAS496B, Tables 1–8.

### Transaction structure

```
START → [Addr+W] → ACK → [CA byte] → ACK → [MSDB] → ACK → [LSDB] → ACK → STOP
```

For consecutive updates to the same channel with the same command, the CA byte
does not need to be re-sent — only MSDB and LSDB are required after the first
transaction. HAL blocking mode sends all 3 bytes every time; this is fine.

### CA (Command/Access) byte

```
Bits [7:4] = C3:C0  — command
Bits [3:0] = SEL3:SEL0 — DAC channel select (0 = channel A … 7 = channel H)
```

| Command (C3:C0) | Code | Operation |
|-----------------|------|-----------|
| Write to input register n | 0x0 | Stage value; do not update output yet |
| Update DAC register n | 0x1 | Transfer staged value to output |
| Write input n, update all | 0x2 | Stage + simultaneous update all channels |
| Write input n, update DAC n | 0x3 | Stage + update this channel immediately |
| Power down/up DAC n | 0x4 | Reduce power on selected channel |
| Software reset | 0xF | Reset all registers to POR state |

Use command **0x3** for normal per-channel updates.

### Data byte packing (12-bit, left-justified in 16 bits)

```
MSDB (DB[15:8]):  D11 D10 D9 D8 D7 D6 D5 D4
LSDB (DB[7:0]):   D3  D2  D1 D0  X  X  X  X   (X = don't care)
```

For a 12-bit value `val`:
```c
uint8_t msb = (val >> 4) & 0xFF;   // D11:D4
uint8_t lsb = (val & 0x0F) << 4;   // D3:D0 in upper nibble
```

**Verified example from datasheet (Example 1):** Write midscale (0x800) to
channel A and update output:
- Address byte: `0x90` (addr 0x48 + write)
- CA byte: `0x00` (command 0x0, channel A = 0) — *note: datasheet example uses
  command 0x0 with global LDAC tied low to auto-update; use 0x3 for software
  control without LDAC*
- MSDB: `0x80` = 0x800 >> 4
- LSDB: `0x00` = (0x800 & 0x0F) << 4 = 0

---

## I2C1 HAL Init

```c
I2C_HandleTypeDef hi2c1;

void MX_I2C1_Init(void) {
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;            // 400 kHz fast mode
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (i2cHandle->Instance == I2C1) {
        __HAL_RCC_I2C1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PB6 = I2C1_SCL, PB7 = I2C1_SDA */
        GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;   // open-drain required for I2C
        GPIO_InitStruct.Pull      = GPIO_NOPULL;        // use external pull-ups on board
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}
```

**Pull-up resistors:** I2C requires external pull-ups on SCL and SDA to VDD.
For 400 kHz Fast mode use 2.2 kΩ. The STM32 internal pull-ups are too weak
(~40 kΩ) for reliable Fast mode operation — fit external resistors on the PCB.

---

## New Helper Functions (replace PWM equivalents in USM.lf preamble)

```c
extern I2C_HandleTypeDef hi2c1;

#define DAC7578_I2C_ADDR    (0x48 << 1)  // 7-bit addr 0x48, shifted for HAL
#define DAC7578_CMD_WRITE_UPDATE  0x3
#define DAC7578_CMD_RESET         0xF
#define DAC7578_DAC_COUNTS        4095    // 12-bit full scale

/* Send one write transaction to a single DAC channel */
void DAC7578_write(uint8_t channel, uint16_t value) {
    uint8_t buf[3];
    buf[0] = (DAC7578_CMD_WRITE_UPDATE << 4) | (channel & 0x0F); // CA byte
    buf[1] = (value >> 4) & 0xFF;   // MSDB: D11:D4
    buf[2] = (value & 0x0F) << 4;   // LSDB: D3:D0 in upper nibble
    HAL_I2C_Master_Transmit(&hi2c1, DAC7578_I2C_ADDR, buf, 3, HAL_MAX_DELAY);
}

/* Replaces convert_duty_cycle_to_pwm_ccr() */
uint16_t convert_duty_cycle_to_dac_counts(float duty_cycle) {
    return (uint16_t)(duty_cycle * (float)DAC7578_DAC_COUNTS);
}

/* Replaces set_motor_pwm_registers() */
void set_motor_dac_outputs(void) {
    for (int i = 0; i < 7; i++) {
        uint16_t counts = convert_duty_cycle_to_dac_counts(usm_duty_cycle_[i]);
        DAC7578_write(i, counts);
    }
}

/* Call once at startup after I2C1 init */
void DAC7578_init(void) {
    uint8_t reset_cmd[3] = {(DAC7578_CMD_RESET << 4), 0x00, 0x00};
    HAL_I2C_Master_Transmit(&hi2c1, DAC7578_I2C_ADDR, reset_cmd, 3, HAL_MAX_DELAY);
    set_motor_dac_outputs(); // set all outputs to 0
}
```

---

## Changes Required Per File (DAC7578 vs AD5628 diff)

The file-by-file changes are identical to the AD5628 plan with these differences:

### `USM.lf`

| AD5628 version | DAC7578 version |
|----------------|-----------------|
| `extern SPI_HandleTypeDef hspi1` | `extern I2C_HandleTypeDef hi2c1` |
| `AD5628_write()` / `AD5628_init()` | `DAC7578_write()` / `DAC7578_init()` |
| Motor 2 enable moved PB3 → PB6 | **PB3 stays as motor 2 enable — no change** |

### `Main.lf`

| AD5628 version | DAC7578 version |
|----------------|-----------------|
| `MX_SPI1_Init()` | `MX_I2C1_Init()` |
| `extern SPI_HandleTypeDef hspi1` | `extern I2C_HandleTypeDef hi2c1` |

### `gpio.c`

| AD5628 version | DAC7578 version |
|----------------|-----------------|
| Remove PB3 from motor enable group, add PB6 | **No changes needed** |
| Remove PB4, PB5, PB7 from GPIO init | Remove PB6, PB7 from GPIO init (configured by `HAL_I2C_MspInit`) |

### `tim.c`

Same as AD5628 plan — no edits required.

---

## Hardware Notes (DAC7578-specific)

**Output reference:** The DAC7578 uses VDD as its reference — output range is
0 V to VDD. There is no internal precision reference. Accuracy tracks the
stability of the supply voltage, so use a low-noise LDO to power the DAC if
output precision matters.

**LDAC pin:** The DAC7578 has an LDAC pin that, when tied low, causes each
write transaction to update the output immediately (equivalent to using command
0x3). Tie LDAC low for simplest operation. Pulling LDAC high and pulsing it
after staging all channels with command 0x0 allows simultaneous update of all
7 outputs — useful if inter-channel phase coherence matters.

**RSTSEL pin:** Controls power-on reset behavior. RSTSEL low → outputs reset
to zero-scale (0 V). RSTSEL high → reset to mid-scale (VDD/2). Tie low for
safe motor behavior on power-up.

**Pull-up resistors:** Fit 2.2 kΩ pull-ups to VDD on both SCL and SDA. Place
them close to the DAC7578 on the PCB, not at the STM32 end.

**Isolation:** Same recommendation as AD5628 — for ground noise isolation, add
a digital isolator (e.g. ADUM1251 or ISO1540 for I2C) and an isolated DC-DC
converter on the DAC side. The ISO1540 from TI is designed specifically for I2C
and handles the open-drain bus topology correctly without needing separate
direction logic.

---

## Adafruit DAC7578 Breakout Board Notes

The [Adafruit DAC7578 breakout](https://learn.adafruit.com/adafruit-dac7578-8-x-channel-12-bit-i2c-dac/pinouts)
is a convenient way to prototype this before committing to a PCB redesign.
Several details differ from a bare chip and affect the wiring and code above.

### What the breakout already provides

- **10 kΩ pull-ups on SDA and SCL** tied to VCC — do not add additional
  external pull-ups. The STM32 GPIO should be configured with `GPIO_NOPULL`
  (already in the init code above).
- **3.3 V / 5 V compatible logic** — power the VCC pin from the STM32's 3.3 V
  rail. No level shifter needed.
- **LDAC and CLR pins** broken out — tie LDAC to GND for immediate output
  update on each write (matches the code above). CLR resets all outputs to
  zero; tie it to VCC to disable it.

### Default I2C address is 0x4C, not 0x48

The breakout ships with AD0 floating, which gives address **0x4C**. Update the
define in the helper code:

```c
// Adafruit breakout default (AD0 floating)
#define DAC7578_I2C_ADDR  (0x4C << 1)

// To use 0x48 instead: solder AD0 pad to GND on the breakout
// #define DAC7578_I2C_ADDR  (0x48 << 1)
```

### Pull-up resistors and I2C speed

The 10 kΩ pull-ups are within spec for **Standard mode (100 kHz)** but are
borderline for **Fast mode (400 kHz)**. The I2C rise-time limit for Fast mode
is 300 ns; with 10 kΩ and ~50 pF of bus capacitance the rise time is
approximately:

```
t_rise ≈ 0.847 × R × C = 0.847 × 10,000 × 50 pF ≈ 424 ns  (exceeds 300 ns limit)
```

For bench testing with short wires this may work in practice, but for
reliability **set the STM32 I2C clock to 100 kHz** while using this breakout:

```c
hi2c1.Init.ClockSpeed = 100000;  // 100 kHz standard mode — safe with 10K pull-ups
```

If 400 kHz is needed for timing reasons, solder a 4.7 kΩ resistor in parallel
with each existing pull-up on the breakout (net resistance ≈ 3.2 kΩ, rise time
≈ 135 ns — within spec). On the final PCB, fit 2.2 kΩ pull-ups as originally
noted.

### Output voltage range

With VCC = 3.3 V, the output range is **0 V to 3.3 V** (VRf defaults to VCC).
Verify that the motor controllers accept a 0–3.3 V reference. If a 0–5 V range
is needed, the breakout has a solder jumper to disconnect VRf from VCC and feed
it from a separate 5 V source, while keeping VCC at 3.3 V for the logic.

### Wiring summary (breakout to STM32 Nucleo)

| Breakout pin | STM32 pin | Note |
|--------------|-----------|------|
| VCC          | 3.3 V     | Powers DAC and sets output reference |
| GND          | GND       | Common ground |
| SDA          | PB7       | I2C1 SDA (AF4) — no extra pull-up needed |
| SCL          | PB6       | I2C1 SCL (AF4) — no extra pull-up needed |
| LDAC         | GND       | Tie low for immediate update on write |
| CLR          | 3.3 V     | Tie high to disable hardware clear |
| DAC0–DAC6    | Motor controller reference inputs 0–6 | |
| DAC7         | (unused)  | Leave floating or connect to a test point |
