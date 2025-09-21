# BME280 ESP32-Arduino Driver

This repository contains an Arduino-compatible driver for the Bosch **BME280** environmental sensor, designed to run on the ESP32 platform. The driver provides easy access to temperature, pressure, and humidity measurements over the I2C interface.

---

## Features

- **Temperature sensing** with ±1 °C accuracy  
- **Pressure sensing** with ±1 hPa absolute accuracy (300–1100 hPa range)  
- **Humidity sensing** with ±3% relative humidity accuracy  
- Supports **I2C interface** using Arduino `Wire` library  
- Configurable **oversampling, filter, and standby settings**  
- Fully implements **compensation formulas** from the Bosch datasheet  
- Provides functions for initialization, calibration, and sensor configuration  

---

## Getting Started

### Requirements

- ESP32 development board  
- Arduino IDE with **ESP32 board support** installed  
- Bosch **BME280** sensor connected via **I2C**  

### Wiring Example (I2C)

| BME280 Pin | ESP32 Pin |
|------------|-----------|
| VCC        | 3.3V      |
| GND        | GND       |
| SDA        | GPIO21    |
| SCL        | GPIO22    |

*(Pins may vary depending on your board – ensure SDA/SCL are configured correctly.)*

---

## Installation

1. Copy `BME280.cpp` and `BME280.h` into your Arduino project folder.  
2. Include the header in your sketch:  

```cpp
#include "BME280.h"
```

---

## Usage Example

```cpp
#include <Wire.h>
#include "BME280.h"

BME280 bme(0x76, Wire);  // Use I2C address 0x76 (or 0x77 depending on module)

void setup() {
    Serial.begin(115200);
    Wire.begin();

    if (bme.Init() == ESP_OK) {
        Serial.println("BME280 initialized successfully.");
    } else {
        Serial.println("BME280 initialization failed!");
        while(1);
    }

    // Configure sensor: Normal mode, x1 oversampling, no filter
    bme.SetOSVals(BME280_MODE_NORMAL, BME280_OS_TEMP_x1, BME280_OS_PRESS_x1, BME280_OS_HUM_x1);
    bme.SetConfig(BME280_STANDBY_1000MS, BME280_FILTER_OFF);
}

void loop() {
    if (bme.GetTemp() == ESP_OK && bme.GetPress() == ESP_OK && bme.GetHum() == ESP_OK) {
        Serial.print("Temperature: ");
        Serial.print(bme.temperature);
        Serial.println(" °C");

        Serial.print("Pressure: ");
        Serial.print(bme.pressure / 100.0f);  // Convert Pa to hPa
        Serial.println(" hPa");

        Serial.print("Humidity: ");
        Serial.print(bme.humidity);
        Serial.println(" %RH");
    } else {
        Serial.println("Failed to read data from BME280.");
    }

    delay(2000);
}
```

---

## API Overview

### Initialization
- `esp_err_t Init()` → Initialize device and load calibration data.

### Data Retrieval
- `esp_err_t GetTemp()` → Reads and compensates temperature (°C).  
- `esp_err_t GetPress()` → Reads and compensates pressure (Pa).  
- `esp_err_t GetHum()` → Reads and compensates humidity (%RH).  

### Configuration
- `esp_err_t SetOSVals(mode, osrs_t, osrs_p, osrs_h)` → Configure oversampling and mode.  
- `esp_err_t SetConfig(t_sb, filter)` → Configure standby time and filter.  

---

## References

- [Bosch BME280 Datasheet](https://cdn-shop.adafruit.com/datasheets/BST-BME280_DS001-10.pdf)  
- Arduino `Wire` library documentation  

---

## Author

**Yair Yamin**  
Date: 17.09.25

---

## License

This driver is released under the MIT License.  
Use it freely for personal and commercial projects.

---
