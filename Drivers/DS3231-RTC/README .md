# DS3231 RTC Driver for ESP32 (Arduino)

## Overview
This repository provides a C++ driver for the **DS3231 Real‑Time Clock (RTC)** module, designed for **ESP32** using the **Arduino core** (`TwoWire`/`Wire`) and **ESP-IDF error codes** (`esp_err_t`).  
The driver supports time & date management, alarms, square‑wave output (SQW), and on‑chip temperature readings.

Reference Datasheet: [DS3231 RTC Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ds3231.pdf)
---

## Features
- **Time & Date**
  - Set/Get time (`hours`, `minutes`, `seconds`) – 24‑hour format
  - Set/Get date (`date`, `month`, `year` 00–99 → 2000–2099)
  - Set/Get **Day Of Week (DOW)** (Sunday=1 … Saturday=7)

- **Alarms**
  - Configure **Alarm 1** and **Alarm 2** with modes:
    - `EveryMinute`, `EveryHour`, `EveryDay`, `Once`, `EveryWeek`
  - Read current alarm configuration
  - Clear both alarm flags (A1F, A2F)

- **Square‑Wave / PWM (SQW)**
  - Configure SQW pin frequency via `OutputPWM(RS2, RS1)`:
    - `00` → 1 Hz
    - `01` → 1.024 kHz
    - `10` → 4.096 kHz
    - `11` → 8.192 kHz

- **Temperature Sensor**
  - Read built‑in sensor at **0.25 °C** resolution

- **Robust API**
  - Return type is **`esp_err_t`** (`ESP_OK`, `ESP_ERR_INVALID_ARG`, `ESP_ERR_INVALID_STATE`, `ESP_ERR_NOT_FINISHED`)
  - Internal BCD conversion helpers

---

## File Structure
- **`DS3231.h`** – public API, register map, types, and class declaration  
- **`DS3231.cpp`** – implementation using `TwoWire` and `esp_err_t`

---

## Requirements
- ESP32 board with Arduino core (via Arduino IDE or PlatformIO)
- `Wire` library (included with Arduino core)
- I²C pull‑ups on SDA/SCL (typ. 4.7 kΩ) on your board

---

## Installation
1. Copy `DS3231.h` and `DS3231.cpp` into your Arduino/PlatformIO project `src/` folder.
2. Include the header in your sketch or C++ module:
```cpp
#include "DS3231.h"
#include <Wire.h>
```

---

## Quick Start

### 1) Wiring
- **ESP32** `GPIO 21` → `SDA` (default Wire SDA)
- **ESP32** `GPIO 22` → `SCL` (default Wire SCL)
- **3V3** and **GND** to module power
- Connect **SQW/INT** to a GPIO if you want to use alarm interrupts

> The DS3231 7‑bit address is typically `0x68`. Pass the 8‑bit form only if your setup expects it; this driver takes the device address you provide verbatim.

### 2) Create and Init the Driver
```cpp
#include <Arduino.h>
#include <Wire.h>
#include "DS3231.h"

void setup() {
  Serial.begin(115200);
  Wire.begin(); // or Wire.begin(SDA_PIN, SCL_PIN);

  // Initial time/date/DOW
  ds3231_time_t t   = { .hours = 12, .minutes = 30, .seconds = 0 };
  ds3231_date_t d   = { .date = 21, .month = 9, .year = 25 }; // 2025
  DOW_t         dow = Sunday;

  // Address: 0x68 (7‑bit) is common for DS3231 modules
  DS3231 rtc(&Wire, 0x68, t, d, dow);

  esp_err_t err = rtc.Init();  // Sets time, date, and DOW
  if (err != ESP_OK) {
    Serial.printf("RTC init failed: %d\n", err);
  }
}

void loop() { }
```

### 3) Read/Write Time & Date
```cpp
// Set time
rtc.SetTime();   // uses values provided in constructor / last set
// Get time
ds3231_time_t nowT = rtc.GetTime();
Serial.printf("%02u:%02u:%02u\n", nowT.hours, nowT.minutes, nowT.seconds);

// Set date
rtc.SetDate();
// Get date
ds3231_date_t nowD = rtc.GetDate();
Serial.printf("%02u/%02u/20%02u\n", nowD.date, nowD.month, nowD.year);

// Day Of Week
rtc.SetDOW();
DOW_t dowNow = rtc.GetDOW();
```

### 4) Alarms
```cpp
// Configure Alarm 1: trigger once at specific date/time
sAlram_t a1 = {
  .hours = 6, .minutes = 30, .seconds = 0,
  .dayOfWeek = Monday, .date = 22
};
esp_err_t e = rtc.SetAlarm1(Once, a1);

// Read back Alarm 1
sAlram_t curA1 = rtc.GetAlarm1();

// Configure Alarm 2: trigger every day at 07:15 (no seconds in A2)
sAlram_t a2 = {
  .hours = 7, .minutes = 15, .seconds = 0,  // seconds ignored by A2
  .dayOfWeek = Tuesday, .date = 0
};
rtc.SetAlarm2(EveryDay, a2);

// Read back Alarm 2
sAlram_t curA2 = rtc.GetAlarm2();

// Clear both alarm flags (call after an alarm interrupt fires)
rtc.CLearAlarmsFlags();
```

### 5) Temperature & Registers
```cpp
float tempC = rtc.GetTemp();
Serial.printf("Temp: %.2f C\n", tempC);

// Control & Status registers
uint8_t ctl = 0, stat = 0;
rtc.GetControlRegister(&ctl);
rtc.ReadStatus(&stat);
// ... modify rtc.Reg[...] via methods and write back if needed:
rtc.WriteStatus();
```

### 6) SQW (Square‑Wave) Output
```cpp
// Set SQW to 1.024 kHz (RS2=0, RS1=1)
rtc.OutputPWM(/*RS2=*/0, /*RS1=*/1);
```

---

## API Summary

### Types
```cpp
typedef struct { uint8_t hours, minutes, seconds; } ds3231_time_t;
typedef struct { uint8_t date, month, year; } ds3231_date_t; // year 00..99 → 2000..2099
typedef enum { Sunday=1, Monday, Tuesday, Wednesday, Thursday, Friday, Saturday } DOW_t;
typedef struct { uint8_t hours, minutes, seconds; DOW_t dayOfWeek; uint8_t date; } sAlram_t;
typedef enum { EveryMinute=1, EveryHour, EveryDay, Once, EveryWeek } sMode_t;
```

### Class
```cpp
class DS3231 {
public:
  DS3231(TwoWire* handle, uint8_t addr,
         const ds3231_time_t time, const ds3231_date_t date, const DOW_t dow);
  ~DS3231();

  esp_err_t     Init();
  esp_err_t     SetTime();
  ds3231_time_t GetTime();
  esp_err_t     SetDOW();
  DOW_t         GetDOW();
  esp_err_t     SetDate();
  ds3231_date_t GetDate();
  esp_err_t     SetAlarm1(sMode_t mode, const sAlram_t alr);
  esp_err_t     SetAlarm2(sMode_t mode, const sAlram_t alr);
  sAlram_t      GetAlarm1();
  sAlram_t      GetAlarm2();
  float         GetTemp();
  esp_err_t     GetControlRegister(uint8_t* ctl);
  esp_err_t     ReadStatus(uint8_t* stat);
  esp_err_t     WriteStatus();
  esp_err_t     OutputPWM(uint8_t RS2, uint8_t RS1);
  esp_err_t     CLearAlarmsFlags();
  esp_err_t     readRegisters(uint8_t reg, uint8_t len);
  esp_err_t     writeRegisters(uint8_t startReg, uint8_t len);
};
```

**Return Codes**
- `ESP_OK` – success
- `ESP_ERR_INVALID_ARG` – invalid time/date/alarm parameters
- `ESP_ERR_INVALID_STATE` – one of the chained operations failed (`VALID` macro)
- `ESP_ERR_NOT_FINISHED` – I²C read/write did not complete as expected

---

## Notes & Tips
- Year is stored as **00–99** and interpreted as **2000–2099**.
- **Alarm 2** does **not** support seconds; set `.seconds` but it will be ignored in matching.
- Use `endTransmission(false)` + `requestFrom` with `Wire` for repeated‑start reads (handled internally).
- Call `CLearAlarmsFlags()` after servicing an alarm interrupt.
- The class owns an internal `Reg[19]` buffer for register transfers.

---

## Author & License
- **Author:** Yair Yamin  
- **License:** MIT‑style or project‑specific (fill in your choice)

---

## Changelog
- **v1.0.0 (2025‑09‑17)**: Initial ESP32 Arduino release.
