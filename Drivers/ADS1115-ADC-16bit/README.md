# ESP32 ADS1115 Driver (Arduino + Wire)

A lightweight ESP32-Arduino driver for the **Texas Instruments ADS1115** 16-bit ΔΣ ADC with PGA and optional comparator/ALERT pin.  
The driver communicates via **I²C** using the Arduino `Wire` library and provides a clean, minimal API for continuous or single-shot conversions, channel selection, gain, sample rate, and comparator configuration.

> **Target audience:** ESP32 developers using the Arduino framework who want direct control over ADS1115 registers without HAL/DMA overhead.

---

## Features

- 16-bit resolution, PGA gains from ±6.144 V … ±0.256 V
- 4 single-ended (AIN0–AIN3) or 2 differential inputs
- Continuous and single-shot conversion modes
- Sample rates from **8 SPS** to **860 SPS**
- Built-in **comparator / ALERT/RDY** support (traditional or window modes)
- Simple C++ class with internal register shadow buffer
- Uses Arduino `TwoWire` (`Wire`/`Wire1`) for I²C transfers

---

## File Layout

```
├─ ADS1115.h
└─ ADS1115.cpp
```

- **ADS1115.h**: Public API, bit-mask definitions, enums, and class declaration.  
- **ADS1115.cpp**: Implementation using Arduino `Wire` API.  

---

## Dependencies

- ESP32 Arduino Core (`Wire.h`, `Arduino.h`)
- I²C peripheral (configured via `Wire.begin()`)
- External pull-ups on SDA/SCL (typically 4.7 kΩ)
- Optional: GPIO for ALERT/RDY pin if comparator is used

---

## Quick Start

### 1) Wire it

- **SDA → SDA**, **SCL → SCL**, **GND**, **VDD (2.0–5.5 V)**  
- **ADDR** selects the 7-bit base address: `0x48..0x4B` (GND/VDD/SDA/SCL)  
- **ALERT/RDY** (optional) → ESP32 GPIO if using comparator/ready  

### 2) Arduino setup

```cpp
#include <Wire.h>
#include "ADS1115.h"

ADS1115 adc(&Wire, 0x48); // default address = 0x48

void setup() {
    Wire.begin();
    Serial.begin(115200);

    // Continuous mode, AIN0 single-ended, ±2.048V, 128 SPS
    if (adc.Init(ADS1115_MODE_CONTINUOUS_MASK, AIN0,
                 ADS1115_PGA_2_048V_MASK, SPS_128) == ESP_OK) {
        Serial.println("ADS1115 initialized.");
    }
}
```

### 3) Read conversions

```cpp
if (adc.ReadConversionReg() == ESP_OK) {
    int16_t raw = (int16_t)adc.Reg[ADS1115_REG_CONVERSION];
    float lsb = 0.0000625f; // for ±2.048V PGA (62.5 µV/LSB)
    float volts = raw * lsb;
    Serial.println(volts, 6);
}
```

---

## Typical Flows

### Continuous conversions on AIN0

```cpp
adc.Init(ADS1115_MODE_CONTINUOUS_MASK, AIN0,
         ADS1115_PGA_2_048V_MASK, SPS_250);

for (;;) {
    adc.ReadConversionReg();
    int16_t raw = (int16_t)adc.Reg[ADS1115_REG_CONVERSION];
    // Convert raw to volts based on PGA
}
```

### Single-shot on AIN1

```cpp
adc.SetSSMode();            // single-shot mode
adc.SetChannel(AIN1);       // select channel
adc.StartSSConv();          // trigger conversion

delay(10); // wait based on data rate
adc.ReadConversionReg();

int16_t raw = (int16_t)adc.Reg[ADS1115_REG_CONVERSION];
```

### Using comparator / ALERT pin

```cpp
// thresholds are raw 16-bit codes
adc.SetThresholds(lo_raw, hi_raw);

adc.Comp_Init(
    ADS1115_COMP_MODE_TRAD_MASK,
    ADS1115_COMP_POL_ACTIVE_LOW_MASK,
    ADS1115_COMP_LAT_NON_LATCHING_MASK,
    ADS1115_COMP_QUE_1_MASK
);

// Connect ALERT/RDY pin to an ESP32 GPIO with interrupt
```

---

## API Overview

> See **ADS1115.h** for full declarations and bit masks.

- `esp_err_t Init(uint16_t mode, sChannel_t channel, uint16_t pga, uint16_t rate)` — configure mode/channel/PGA/SPS.  
- `esp_err_t ReadConfigReg()` — read CONFIG register.  
- `esp_err_t ReadConversionReg()` — read CONVERSION register.  
- `esp_err_t SetChannel(sChannel_t channel)` — update input channel.  
- `esp_err_t SetSampleRate(sSampleRate_t rate)` — set sample rate.  
- `esp_err_t SetSSMode()` — switch to single-shot mode.  
- `esp_err_t StartSSConv()` — trigger single conversion.  
- `esp_err_t SetThresholds(uint16_t lo, uint16_t hi)` — set comparator thresholds.  
- Comparator helpers: `Comp_Init`, `Comp_SetMode`, `Comp_SetPol`, `Comp_SetLat`, `Comp_SetQue`.  

---

## Notes & Gotchas

- **Register order:** ADS1115 is MSB-first. The driver already handles byte ordering.  
- **Channel enum:** The enum includes `Ain3` (lowercase `in`) — watch for typo sensitivity.  
- **I²C address:** Pass the **7-bit** address (0x48 default). The `Wire` library internally shifts it for read/write.  
- **Delays:** At low data rates (e.g. 8 SPS), allow enough time for conversions before reading.  

---

## Author

**Yair Yamin**  
Date: 17.09.25

---

## References

- [ADS1115 Datasheet](https://www.ti.com/lit/ds/symlink/ads1115.pdf)
- [STM32 HAL Documentation](https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html)

---

## License

This driver is provided as-is for educational and commercial use. Please refer to your project's license terms.