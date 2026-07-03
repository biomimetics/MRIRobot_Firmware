# SPI Interface Availability — STM32F446RE

## Summary

Of the four hardware SPI peripherals on the STM32F446RE, **only SPI4 has a
complete set of free pins**. All others are fully blocked by existing GPIO,
timer, and UART assignments. SPI4's pins are on the PE port, which is accessible
only via the morpho connectors (CN7/CN10), not the Arduino-compatible headers.

---

## Why SPI1–SPI3 Are Blocked

### SPI1 (AF5)

| Pin set | Pins | Conflict |
|---------|------|----------|
| Primary | PA4 (NSS), PA5 (SCK), PA6 (MISO), PA7 (MOSI) | All used as motor direction GPIO outputs (`gpio.c`, `USM.lf`) |
| Secondary | PA15 (NSS), PB3 (SCK), PB4 (MISO), PB5 (MOSI) | PA15 = encoder GPIO input; PB3 = motor enable; PB4/PB5 = TIM3 CH1/CH2 PWM |

### SPI2 (AF5)

| Pin set | Pins | Conflict |
|---------|------|----------|
| Set A | PB9 (NSS), PB10 (SCK), PC2 (MISO), PC3 (MOSI) | PB9/PB10 = motor enable outputs; PC2/PC3 = input switches |
| Set B | PB12 (NSS), PB13 (SCK), PB14 (MISO), PB15 (MOSI) | All = motor enable/direction GPIO outputs |

### SPI3 (AF6)

| Pin set | Pins | Conflict |
|---------|------|----------|
| Primary | PC10 (SCK), PC11 (MISO), PC12 (MOSI) | PC10/PC11 = USART3 TX/RX (encoder QDEC comms) |
| Secondary | PB3 (SCK), PB4 (MISO), PB5 (MOSI) | PB3 = motor enable; PB4/PB5 = TIM3 CH1/CH2 PWM |

NSS options for SPI3 (PA4, PA15) are also both taken.

---

## SPI4 — Available (AF5, APB2 bus)

SPI4 has two complete pin sets, neither of which is assigned anywhere in the
current firmware. Both sets are on port E, which is unused by the application.

### Pin set A (lower PE numbers)

| Function | Pin | AF  |
|----------|-----|-----|
| NSS      | PE4 | AF5 |
| SCK      | PE2 | AF5 |
| MISO     | PE5 | AF5 |
| MOSI     | PE6 | AF5 |

### Pin set B (higher PE numbers)

| Function | Pin  | AF  |
|----------|------|-----|
| NSS      | PE11 | AF5 |
| SCK      | PE12 | AF5 |
| MISO     | PE13 | AF5 |
| MOSI     | PE14 | AF5 |

Pin set B (PE11–PE14) groups all four signals on consecutive pins, which is
convenient for layout. Either set works electrically; the choice depends on
which morpho pins are most accessible on the PCB.

**Morpho connector:** All PE pins are on the CN10 morpho connector on the
Nucleo F446RE. Verify exact pin numbers against the board pinout table in
ST document UM1724 (Nucleo-64 user manual) before routing.

**NSS note:** Hardware NSS (managed by the SPI peripheral) requires the NSS
pin to be configured as AF5. For multi-device buses or tighter software
control, NSS can instead be a plain GPIO output on any free pin, with
`SPI_NSS_SOFT` set in the HAL init — this frees PE4/PE11 for other use.

---

## Clock Speed

SPI4 is on the APB2 bus (unlike SPI1–SPI3 which are on APB1 except SPI1).

```
APB2 clock = HCLK / APB2CLKDivider = 84 MHz / 1 = 84 MHz
SPI4 input clock = 84 MHz

Max SPI4 bit rate = 84 MHz / BaudRatePrescaler
```

| Prescaler define            | SPI bit rate |
|-----------------------------|--------------|
| `SPI_BAUDRATEPRESCALER_2`   | 42.0 MHz     |
| `SPI_BAUDRATEPRESCALER_4`   | 21.0 MHz     |
| `SPI_BAUDRATEPRESCALER_8`   | 10.5 MHz     |
| `SPI_BAUDRATEPRESCALER_16`  | 5.25 MHz     |
| `SPI_BAUDRATEPRESCALER_32`  | 2.625 MHz    |
| `SPI_BAUDRATEPRESCALER_64`  | 1.313 MHz    |
| `SPI_BAUDRATEPRESCALER_128` | 656 kHz      |
| `SPI_BAUDRATEPRESCALER_256` | 328 kHz      |

---

## DMA Streams for SPI4

SPI4 can use DMA2. Check for conflicts with the DMA streams already assigned to
USART1 (DMA2_Stream5/7) and USART6 (DMA2_Stream1/6) before choosing streams.

| Direction | DMA2 stream options    | Channel |
|-----------|------------------------|---------|
| RX        | Stream 0 or Stream 3   | CH4     |
| TX        | Stream 1               | CH4     |

Stream 1 is already used by `hdma_usart6_rx` (channel 5), but on a different
channel — DMA streams are shared by channel so verify no conflict. Stream 0 and
Stream 3 for RX appear free.

---

## HAL Init Skeleton (pin set B, software NSS)

```c
SPI_HandleTypeDef hspi4;

void MX_SPI4_Init(void) {
    hspi4.Instance               = SPI4;
    hspi4.Init.Mode              = SPI_MODE_MASTER;
    hspi4.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi4.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi4.Init.CLKPolarity       = SPI_POLARITY_LOW;   // CPOL=0
    hspi4.Init.CLKPhase          = SPI_PHASE_1EDGE;    // CPHA=0 → Mode 0
    hspi4.Init.NSS               = SPI_NSS_SOFT;       // software chip select
    hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // 5.25 MHz
    hspi4.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi4.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi4.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi4);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (spiHandle->Instance == SPI4) {
        __HAL_RCC_SPI4_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();

        /* PE12=SCK, PE13=MISO, PE14=MOSI  (NSS handled in software) */
        GPIO_InitStruct.Pin       = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* PE11 as software NSS — plain output, pulled high initially */
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
        GPIO_InitStruct.Pin   = GPIO_PIN_11;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    }
}
```

Call `MX_SPI4_Init()` from `stm32_init()` in `Main.lf` alongside the existing
`MX_UART4_Init()` / `MX_TIM3_Init()` calls.

---

## Fallback: Software (Bitbang) SPI

If morpho connector access is not feasible on the PCB, two GPIO pins are
confirmed free on ports already broken out:

| Pin  | Notes |
|------|-------|
| PB11 | Not assigned to any peripheral or GPIO function in the current firmware |
| PC12 | Not assigned; also the UART5 TX alternate function (AF8) if UART5 is ever needed |

A third free GPIO would be needed for MISO (if bidirectional) and a fourth for
CS. Bitbang SPI is simpler to set up but is blocking and limited to a few MHz
before timing issues arise at the CPU's clock rate.
