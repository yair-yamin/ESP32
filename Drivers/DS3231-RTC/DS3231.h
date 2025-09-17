#ifndef DS3231_H
#define DS3231_H

#include <Arduino.h>
#include "Wire.h"

/*------------------- Macros ---------------------------*/
#define VALID(x) if((x) != ESP_OK) { return ESP_ERR_INVALID_STATE; }
#define decToBCD(value) (((value / 10) << 4) | (value % 10))
#define BCDToDec(value) (((value >> 4) * 10) + (value & 0x0F))

/*------------------- Regester Address Defines ---------------------------*/
#define DS3231_REG_SECONDS 0x00
#define DS3231_REG_MINUTES 0x01
#define DS3231_REG_HOURS   0x02
#define DS3231_REG_DAY     0x03
#define DS3231_REG_DATE    0x04
#define DS3231_REG_MONTH   0x05
#define DS3231_REG_YEAR    0x06
#define DS3231_REG_ALARM1_SECONDS 0x07
#define DS3231_REG_ALARM1_MINUTES 0x08
#define DS3231_REG_ALARM1_HOURS   0x09
#define DS3231_REG_ALARM1_DAYDATE 0x0A
#define DS3231_REG_ALARM2_MINUTES 0x0B
#define DS3231_REG_ALARM2_HOURS   0x0C
#define DS3231_REG_ALARM2_DAYDATE 0x0D
#define DS3231_REG_CONTROL 0x0E
#define DS3231_REG_STATUS 0x0F
#define DS3231_REG_AGING 0x10
#define DS3231_REG_TEMP_MSB 0x11
#define DS3231_REG_TEMP_LSB 0x12

/************************ Bit Mask defines ********************************/
#define ALRAM_DAY_MASK 0b01000000 // Mask for day of the week in Alarm registers
#define INTR_MODE_MASK 0b00000100 // Interrupt mode for SQW pin  
#define ALARM1_MASK 0b00000001 // Alarm 1 Interrupt enable 
#define ALARM2_MASK 0b00000010 // Alarm 2 Interrupt enable
#define PWM_MODE_MASK 0b00000100 // PWM mode for SQW pin

/************************ Driver Structs ********************************/
typedef struct {
    uint8_t hours;    // 0-23
    uint8_t minutes;  // 0-59
    uint8_t seconds;  // 0-59
}ds3231_time_t;

typedef struct {
    uint8_t date;  // 1-31
    uint8_t month; // 1-12
    uint8_t year;  // 0-99 (0=2000, 1=2001, ..., 99=2099)
}ds3231_date_t;

typedef enum {
    Sunday = 1,
    Monday = 2,
    Tuesday = 3,
    Wednesday = 4,
    Thursday = 5,
    Friday = 6,
    Saturday = 7
}DOW_t;

typedef struct {
    uint8_t hours;    // 0-23
    uint8_t minutes;  // 0-59
    uint8_t seconds;  // 0-59
    DOW_t dayOfWeek;  // Day of the week
    uint8_t date;     // Date
}sAlram_t;

typedef enum {
    EveryMinute = 1,
    EveryHour = 2,
    EveryDay = 3,
    Once = 4,
    EveryWeek = 5,
}sMode_t;
/************************ DS3231 Handle class ********************************/

class DS3231 {
public:
    DS3231(TwoWire* handle,uint8_t addr,const ds3231_time_t time,const ds3231_date_t date,const DOW_t dow);
    ~DS3231();
    esp_err_t Init();
    esp_err_t SetTime(void);
    ds3231_time_t GetTime(void);
    esp_err_t SetDOW(void);
    DOW_t GetDOW(void);
    esp_err_t SetDate(void);
    ds3231_date_t GetDate(void);
    esp_err_t SetAlarm1(sMode_t mode,const sAlram_t alr);
    esp_err_t SetAlarm2(sMode_t mode,const sAlram_t alr);
    sAlram_t GetAlarm1(void);
    sAlram_t GetAlarm2(void);
    float GetTemp(void);
    esp_err_t GetControlRegister(uint8_t *ctl);
    esp_err_t ReadStatus(uint8_t * stat);
    esp_err_t WriteStatus(void);
    esp_err_t OutputPWM(uint8_t RS2, uint8_t RS1);
    esp_err_t CLearAlarmsFlags(void);
    esp_err_t readRegisters(uint8_t reg,uint8_t len);
    esp_err_t writeRegisters(uint8_t startReg, uint8_t len);
private:
    TwoWire* i2c_handle;
    uint8_t i2c_address;
    ds3231_time_t time;
    ds3231_date_t date;
    DOW_t dayOfWeek;
    uint8_t *Reg; // Registers buffer
    sAlram_t alarm1;
    sAlram_t alarm2;
    float temp;
};


#endif // DS3231_H