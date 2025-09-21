#ifndef BME280_H
#define BME280_H
#include <Wire.h>
#include <stdint.h>
/*------------------- Regester Address Defines ---------------------------*/
#define BME280_ID_REG 0xD0
#define BME280_RESET_REG 0xE0
#define BME280_CTRL_HUM_REG 0xF2
#define BME280_STATUS_REG 0xF3
#define BME280_CTRL_MEAS_REG 0xF4
#define BME280_CONFIG_REG 0xF5
#define BME280_PRESS_MSB_REG 0xF7
#define BME280_PRESS_LSB_REG 0xF8
#define BME280_PRESS_XLSB_REG 0xF9
#define BME280_TEMP_MSB_REG 0xFA
#define BME280_TEMP_LSB_REG 0xFB
#define BME280_TEMP_XLSB_REG 0xFC
#define BME280_HUM_MSB_REG 0xFD
#define BME280_HUM_LSB_REG 0xFE
/************************ Bit Mask defines ********************************/
#define BME280_OS_HUM_SKIP 0x00
#define BME280_OS_HUM_x1   0x01
#define BME280_OS_HUM_x2   0x02
#define BME280_OS_HUM_x4   0x03
#define BME280_OS_HUM_x8   0x04
#define BME280_OS_HUM_x16  0x05
#define BME280_OS_TEMP_SKIP 0x00
#define BME280_OS_TEMP_x1   0x01
#define BME280_OS_TEMP_x2   0x02
#define BME280_OS_TEMP_x4   0x03
#define BME280_OS_TEMP_x8   0x04
#define BME280_OS_TEMP_x16  0x05
#define BME280_OS_PRESS_SKIP 0x00
#define BME280_OS_PRESS_x1   0x01
#define BME280_OS_PRESS_x2   0x02
#define BME280_OS_PRESS_x4   0x03
#define BME280_OS_PRESS_x8   0x04
#define BME280_OS_PRESS_x16  0x05
#define BME280_MODE_SLEEP 0x00
#define BME280_MODE_FORCED 0x01
#define BME280_MODE_NORMAL 0x03
#define BME280_STANDBY_0_5MS 0x00
#define BME280_STANDBY_62_5MS 0x01
#define BME280_STANDBY_125MS 0x02
#define BME280_STANDBY_250MS 0x03
#define BME280_STANDBY_500MS 0x04
#define BME280_STANDBY_1000MS 0x05
#define BME280_STANDBY_10MS 0x06
#define BME280_STANDBY_20MS 0x07
#define BME280_FILTER_OFF 0x00
#define BME280_FILTER_x2 0x01
#define BME280_FILTER_x4 0x02
#define BME280_FILTER_x8 0x03
#define BME280_FILTER_x16 0x04

/************************ Driver Structs ********************************/
typedef int32_t   BME280_S32_t;
typedef uint32_t  BME280_U32_t;
typedef int64_t   BME280_S64_t;
typedef uint16_t  BME280_U16_t;
typedef int16_t  BME280_S16_t;
typedef uint8_t   BME280_U8_t;
typedef int8_t    BME280_S8_t;

typedef struct {
    BME280_U16_t dig_T1;
    BME280_S16_t dig_T2;
    BME280_S16_t dig_T3;
    BME280_U16_t dig_P1;
    BME280_S16_t dig_P2;
    BME280_S16_t dig_P3;
    BME280_S16_t dig_P4;
    BME280_S16_t dig_P5;
    BME280_S16_t dig_P6;
    BME280_S16_t dig_P7;
    BME280_S16_t dig_P8;
    BME280_S16_t dig_P9;
    BME280_U8_t  dig_H1;
    BME280_S16_t dig_H2;
    BME280_U8_t  dig_H3;
    BME280_S16_t dig_H4;
    BME280_S16_t dig_H5;
    BME280_S8_t  dig_H6;
}BME280_Compensations_t;

typedef struct {
    uint8_t id_reg;
    uint8_t reset_reg;
    uint8_t ctrl_hum_reg;
    uint8_t status_reg;
    uint8_t ctrl_meas_reg;
    uint8_t config_reg;
    uint8_t press_msb_reg;
    uint8_t press_lsb_reg;
    uint8_t press_xlsb_reg;
    uint8_t temp_msb_reg;
    uint8_t temp_lsb_reg;
    uint8_t temp_xlsb_reg;
    uint8_t hum_msb_reg;
    uint8_t hum_lsb_reg;
} BME280_RegMap_t;

class BME280 {
   public:
    BME280(uint8_t address, TwoWire &wirePort = Wire);
    esp_err_t Init();
    esp_err_t CalCompensationParams();
    esp_err_t GetTemp();
    esp_err_t GetPress();
    esp_err_t GetHum();
    esp_err_t SetOSVals(uint8_t mode, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h);
    esp_err_t SetConfig(uint8_t t_sb, uint8_t filter);
    esp_err_t readRegisters(uint8_t reg, uint8_t* data, uint16_t len);
    esp_err_t writeRegisters(uint8_t reg, uint8_t* data, uint16_t len);
   private:
    TwoWire* i2c_handle;
    uint8_t I2C_address;
    float temperature;
    float pressure;
    float humidity;
    BME280_Compensations_t Comp;
    BME280_RegMap_t Reg;
};

/*------------------- Function Declarations ---------------------------*/

BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T,BME280_Compensations_t comp);
BME280_U32_t BME280_compensate_P_int64(BME280_S32_t adc_P,BME280_Compensations_t comp);
BME280_U32_t BME280_compensate_H_int32(BME280_S32_t adc_H,BME280_Compensations_t comp);

#endif // BME280_H