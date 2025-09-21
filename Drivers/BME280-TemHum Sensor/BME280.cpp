#include "BME280.h"
/**
 ******************************************************************************
 * @file    BME280.h
 * @author  Yair Yamin
 * @date    17.09.25
 * @brief   Header file for BME280 driver for ESP32-Arduino
 * @see     https://www.ti.com/lit/ds/symlink/ads1115.pdf
 * @details This header file provides definitions, structures, and function
 *          declarations for interfacing with the BME280 temperature, humidity, and
 *          pressure sensor and reading data from the
 *          I2C using Wire library .
 *
 * Key Features:
 * - Temperature, pressure, and humidity sensing in a single package
 * - Temperature: ±1°C accuracy, pressure: ±1 hPa absolute accuracy
 * - Humidity: ±3% relative humidity accuracy
 * - Pressure range: 300-1100 hPa
 * - Multiple power modes and oversampling settings
 * - I2C and SPI digital interfaces (I2C implementation in this driver)
 *
 ******************************************************************************
 */

/* ========================== Global Variables ============================ */
BME280_S32_t t_fine;

 /* ========================== Macros ============================ */
static inline uint16_t u16(uint8_t lsb, uint8_t msb) { return (uint16_t)lsb | ((uint16_t)msb << 8); }
static inline  int16_t s16(uint8_t lsb, uint8_t msb) { return (int16_t) u16(lsb, msb); }

/* ========================== Function Definitions ============================ */

/**
 * @brief Calculate raw temperature reading using calibration data
 * @param adc_T Raw temperature reading from sensor
 * @param comp BME280 calibration compensation parameters
 * @return BME280_S32_t Compensated temperature value in DegC * 100
 */
BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T,BME280_Compensations_t comp)
{
    BME280_S32_t var1, var2, T;
    var1  = ((((adc_T >> 3) - ((BME280_S32_t)comp.dig_T1 << 1))) * ((BME280_S32_t)comp.dig_T2)) >> 11;
    var2  = (((((adc_T >> 4) - ((BME280_S32_t)comp.dig_T1)) *
              ((adc_T >> 4) - ((BME280_S32_t)comp.dig_T1))) >> 12) *
              ((BME280_S32_t)comp.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T  = (t_fine * 5 + 128) >> 8;
    return T;
}

/**
 * @brief Calculate raw pressure reading using calibration data
 * @param adc_P Raw pressure reading from sensor
 * @param comp BME280 calibration compensation parameters
 * @return BME280_U32_t Compensated pressure value in Pa * 256
 */
BME280_U32_t BME280_compensate_P_int64(BME280_S32_t adc_P,BME280_Compensations_t comp)
{
    BME280_S64_t var1, var2, p;
    var1 = ((BME280_S64_t)t_fine) - 128000;
    var2 = var1 * var1 * (BME280_S64_t)comp.dig_P6;
    var2 = var2 + ((var1 * (BME280_S64_t)comp.dig_P5) << 17);
    var2 = var2 + (((BME280_S64_t)comp.dig_P4) << 35);
    var1 = ((var1 * var1 * (BME280_S64_t)comp.dig_P3) >> 8) + ((var1 * (BME280_S64_t)comp.dig_P2) << 12);
    var1 = (((((BME280_S64_t)1) << 47) + var1)) * ((BME280_S64_t)comp.dig_P1) >> 33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((BME280_S64_t)comp.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((BME280_S64_t)comp.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)comp.dig_P7) << 4);
    return (BME280_U32_t)p;
}

/**
 * @brief Calculate raw humidity reading using calibration data
 * @param adc_H Raw humidity reading from sensor
 * @param comp BME280 calibration compensation parameters
 * @return BME280_U32_t Compensated humidity value in %RH * 1024
 */
BME280_U32_t BME280_compensate_H_int32(BME280_S32_t adc_H,BME280_Compensations_t comp)
{
    BME280_S32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((BME280_S32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)comp.dig_H4) << 20) -
                    (((BME280_S32_t)comp.dig_H5) * v_x1_u32r)) + 
                   ((BME280_S32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((BME280_S32_t)comp.dig_H6)) >> 10) *
                      (((v_x1_u32r * ((BME280_S32_t)comp.dig_H3)) >> 11) +
                       ((BME280_S32_t)32768))) >> 10) +
                    ((BME280_S32_t)2097152)) * ((BME280_S32_t)comp.dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r -
                 (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                   ((BME280_S32_t)comp.dig_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (BME280_U32_t)(v_x1_u32r >> 12);
}

/**
 * @brief Constructor for BME280 sensor class
 * @param address I2C address of the BME280 sensor
 * @param wirePort Reference to TwoWire I2C interface
 */
BME280::BME280(uint8_t address, TwoWire &wirePort) {
    I2C_address = address;
    i2c_handle = &wirePort;
}

/**
 * @brief Initialize the BME280 sensor
 * @return esp_err_t ESP_OK on success, ESP_FAIL if device ID doesn't match
 * @details Reads the device ID and calculates compensation parameters
 */
esp_err_t BME280::Init() {
   readRegisters(BME280_ID_REG, &Reg.id_reg, 1);
    if (Reg.id_reg != 0x60) {
            return ESP_FAIL; // Device ID does not match BME280
    }
    return CalCompensationParams();
}

/**
 * @brief Calculate compensation parameters from sensor calibration data
 * @return esp_err_t ESP_OK on success, error code on failure
 * @details Reads calibration data from sensor registers and calculates compensation
 *          parameters for temperature, pressure, and humidity measurements
 */
esp_err_t BME280::CalCompensationParams() {
   esp_err_t status;
    uint8_t  calibA[26] = {0};
    uint8_t  calibB[7] = {0};
    

    status = readRegisters(0x88,calibA,26);
    if(status != ESP_OK) return status;
    status = readRegisters(0xE1,calibB,7);
    if(status != ESP_OK) return status;
    /* Temperature coefficients (Table 16) */
    uint16_t dig_T1 = u16(calibA[0],  calibA[1]);   // 0x88 / 0x89  (unsigned)
    int16_t dig_T2 = s16(calibA[2],  calibA[3]);   // 0x8A / 0x8B  (signed)
    int16_t dig_T3 = s16(calibA[4],  calibA[5]);   // 0x8C / 0x8D  (signed)

    /* Pressure coefficients (Table 16) */
    Comp.dig_P1 = u16(calibA[6],  calibA[7]);   // 0x8E / 0x8F  (unsigned)
    Comp.dig_P2 = s16(calibA[8],  calibA[9]);   // 0x90 / 0x91  (signed)
    Comp.dig_P3 = s16(calibA[10], calibA[11]);  // 0x92 / 0x93  (signed)
    Comp.dig_P4 = s16(calibA[12], calibA[13]);  // 0x94 / 0x95  (signed)
    Comp.dig_P5 = s16(calibA[14], calibA[15]);  // 0x96 / 0x97  (signed)
    Comp.dig_P6 = s16(calibA[16], calibA[17]);  // 0x98 / 0x99  (signed)
    Comp.dig_P7 = s16(calibA[18], calibA[19]);  // 0x9A / 0x9B  (signed)
    Comp.dig_P8 = s16(calibA[20], calibA[21]);  // 0x9C / 0x9D  (signed)
    Comp.dig_P9 = s16(calibA[22], calibA[23]);  // 0x9E / 0x9F  (signed)

    /* Humidity coefficients (Table 16 + notes on H4/H5 packing) */
    Comp.dig_H1 = calibA[25];                   // 0xA1 (unsigned 8-bit)
    Comp.dig_H2 = s16(calibB[0],  calibB[1]);   // 0xE1 / 0xE2  (signed)
    Comp.dig_H3 = calibB[2];                    // 0xE3 (unsigned 8-bit)

    /* H4: [11:4] in 0xE4, [3:0] in 0xE5[3:0]  → sign-extended 12-bit => int16_t */
    Comp.dig_H4 = (int16_t)(( (int16_t)calibB[3] << 4) | (calibB[4] & 0x0F));
    /* H5: [3:0] in 0xE5[7:4], [11:4] in 0xE6   → sign-extended 12-bit => int16_t */
    Comp.dig_H5 = (int16_t)(( (int16_t)calibB[5] << 4) | (calibB[4] >> 4));
    Comp.dig_H6 = (int8_t)calibB[6];           // 0xE7 (signed 8-bit)

    return ESP_OK;
}

/**
 * @brief Read and calculate the temperature
 * @return esp_err_t ESP_OK on success, error code on failure
 * @details Reads raw temperature data from sensor and applies compensation
 *          to calculate actual temperature in degrees Celsius
 */
esp_err_t BME280::GetTemp() {
    esp_err_t status;
    BME280_S32_t adc_T;

    status = readRegisters(BME280_TEMP_MSB_REG, &Reg.temp_msb_reg, 3);
    if(status != ESP_OK) return status;

    adc_T = (BME280_S32_t)(((uint32_t)(Reg.temp_msb_reg) << 12) | ((uint32_t)(Reg.temp_lsb_reg) << 4) | ((uint32_t)Reg.temp_xlsb_reg >> 4));

    temperature = BME280_compensate_T_int32(adc_T,Comp);
    temperature /= 100.0f; // convert to degC

    return ESP_OK;
}

/**
 * @brief Read and calculate the pressure
 * @return esp_err_t ESP_OK on success, error code on failure
 * @details Reads raw pressure data from sensor and applies compensation
 *          to calculate actual pressure in Pascals
 */
esp_err_t BME280::GetPress() {
    esp_err_t status;
    BME280_S32_t adc_P;

    status = readRegisters(BME280_PRESS_MSB_REG, &Reg.press_msb_reg, 3);
    if(status != ESP_OK) return status;

    adc_P = (BME280_S32_t)(((uint32_t)(Reg.press_msb_reg) << 12) | ((uint32_t)(Reg.press_lsb_reg) << 4) | ((uint32_t)Reg.press_xlsb_reg >> 4));

    pressure = BME280_compensate_P_int64(adc_P,Comp);
    pressure /= 256.0f; // convert to Pa

    return ESP_OK;
}

/**
 * @brief Read and calculate the humidity
 * @return esp_err_t ESP_OK on success, error code on failure
 * @details Reads raw humidity data from sensor and applies compensation
 *          to calculate actual relative humidity in percentage
 */
esp_err_t BME280::GetHum() {
    esp_err_t status;
    BME280_S32_t adc_H;

    status = readRegisters(BME280_HUM_MSB_REG, &Reg.hum_msb_reg, 2);
    if(status != ESP_OK) return status;

    adc_H = (BME280_S32_t)(((uint32_t)(Reg.hum_msb_reg) << 8) | ((uint32_t)Reg.hum_lsb_reg));

    humidity = BME280_compensate_H_int32(adc_H,Comp);
    humidity /= 1024.0f; // convert to %RH

    return ESP_OK;
}

/**
 * @brief Set oversampling values and mode for the sensor
 * @param mode Operating mode (sleep, forced, or normal)
 * @param osrs_t Temperature oversampling setting
 * @param osrs_p Pressure oversampling setting
 * @param osrs_h Humidity oversampling setting
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t BME280::SetOSVals(uint8_t mode, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h) {
    esp_err_t status;
    uint8_t ctrl_meas = (osrs_t << 5) | (osrs_p << 2) | mode;
    uint8_t ctrl_hum = osrs_h;

    status = writeRegisters(BME280_CTRL_HUM_REG, &ctrl_hum, 1);
    if(status != ESP_OK) return status;
    status = writeRegisters(BME280_CTRL_MEAS_REG, &ctrl_meas, 1);
    if(status != ESP_OK) return status;

    return ESP_OK;
}

/**
 * @brief Configure the sensor's standby time and filter settings
 * @param t_sb Standby time between measurements in normal mode
 * @param filter IIR filter coefficient
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t BME280::SetConfig(uint8_t t_sb, uint8_t filter) {
    esp_err_t status;
    uint8_t config = (t_sb << 5) | (filter << 2);

    status = writeRegisters(BME280_CONFIG_REG, &config, 1);
    if(status != ESP_OK) return status;

    return ESP_OK;
}