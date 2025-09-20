#include "ADS1115.h"
/**
 ******************************************************************************
 * @file    ADS1115.h
 * @author  Yair Yamin
 * @date    17.09.25
 * @brief   Header file for ADS1115 16-bit ADC driver for ESP32-ARDUINO using I2C
 * @see     https://www.ti.com/lit/ds/symlink/ads1115.pdf
 * @details This header file provides definitions, structures, and function
 *          declarations for interfacing with the ADS1115 16-bit analog-to-digital
 *          converter via I2C using Wire library .
 *
 * Key Features:
 * - 16-bit resolution ADC with programmable gain amplifier (PGA)
 * - 4 single-ended or 2 differential input channels
 * - Programmable data rate from 8 to 860 samples per second
 * - Single-shot and continuous conversion modes
 *
 ******************************************************************************
 */

/* ========================== Function Definitions ============================ */
/**
 * @brief Constructor for ADS1115 class
 * @param i2c Pointer to TwoWire instance for I2C communication
 * @param address I2C address of the ADS1115 device
 * @details Initializes the ADS1115 object with I2C interface and device address
 */
ADS1115::ADS1115(TwoWire* i2c, uint8_t address):
i2c_handle(i2c), i2c_address(address) {
   memset(Reg,0,sizeof(Reg)); // Clear register buffer
}

/**
 * @brief Destructor for ADS1115 class
 * @details Deallocates the dynamically allocated register buffer
 */
ADS1115::~ADS1115() {
}

/**
 * @brief Initialize the ADS1115 with specified configuration
 * @param mode Operating mode (continuous or single-shot)
 * @param channel Input channel selection
 * @param pga Programmable gain amplifier setting
 * @param rate Data rate configuration
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ADS1115::Init(uint16_t mode,sChannel_t channel,uint16_t pga,uint16_t rate)
{
    esp_err_t status;
    memset(Reg,0,sizeof(Reg)); // Clear register buffer
    uint16_t channel_config = 0;
    channel_config = (uint16_t)channel << 12;
    Reg[ADS1115_REG_CONFIG] = channel_config | pga | mode | rate | ADS1115_COMP_QUE_DISABLE_MASK;
    status = writeRegisters(ADS1115_REG_CONFIG);
    return status; 
}


/**
 * @brief Write data to ADS1115 registers
 * @param startReg Starting register address to write to
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_FINISHED on I2C error
 * @details Writes 2 bytes of data from internal Reg buffer to the specified register
 */
esp_err_t ADS1115::writeRegisters(uint8_t startReg)
{
    ptr_reg = startReg;
    i2c_handle->beginTransmission(this->i2c_address);
    i2c_handle->write(ptr_reg); // set the register pointer

    for (uint8_t i = 0; i < 2; i++) {
        i2c_handle->write(Reg[startReg + i]); // write each byte
    }

    uint8_t result = i2c_handle->endTransmission(); // 0 = success

    if (result == 0) {
        return ESP_OK;
    } 
    return ESP_ERR_NOT_FINISHED;
}


/**
 * @brief Read data from ADS1115 registers
 * @param reg Register address to read from
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_FINISHED on I2C error,
 *         ESP_ERR_INVALID_RESPONSE if incorrect number of bytes read
 * @details Reads 2 bytes of data from the specified register into internal Reg buffer
 */
esp_err_t ADS1115::readRegisters(uint8_t reg)
{
    ptr_reg = reg;
    i2c_handle->beginTransmission(this->i2c_address);
    i2c_handle->write(ptr_reg); 
    uint8_t result = i2c_handle->endTransmission(); // 0 = success

    if (result != 0) {
        return ESP_ERR_NOT_FINISHED;
    }

    size_t bytesRead = i2c_handle->requestFrom(this->i2c_address, 2);
    if (bytesRead != 2) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (i2c_handle->available() ) {
        uint8_t temp[2] = {0};
        i2c_handle->readBytes(temp, bytesRead);
        Reg[reg] = (temp[0] << 8) | temp[1];
        return ESP_OK;
    }

    return ESP_OK;
}

/**
 * @brief Read the configuration register
 * @return esp_err_t ESP_OK on success, error code otherwise
 * @details Reads the current configuration register value into internal buffer
 */
esp_err_t ADS1115::ReadConfigReg()
{
    return readRegisters(ADS1115_REG_CONFIG);
}

/**
 * @brief Read the conversion register
 * @return esp_err_t ESP_OK on success, error code otherwise
 * @details Reads the latest conversion result into internal buffer
 */
esp_err_t ADS1115::ReadConversionReg()
{
    return readRegisters(ADS1115_REG_CONVERSION);
}

/**
 * @brief Set the input channel configuration
 * @param channel Input channel selection (single-ended or differential)
 * @return esp_err_t ESP_OK on success, error code otherwise
 * @details Updates the MUX bits in the configuration register to select input channel
 */
esp_err_t ADS1115::SetChannel(sChannel_t channel)
{
    esp_err_t status;
    status = ReadConfigReg();
    if ((status != ESP_OK))
    {
        return status;
    }
    this->channel = channel;
    uint16_t channel_config = (uint16_t)channel << 12;
    Reg[ADS1115_REG_CONFIG] &= ~0x7000; // Clear MUX bits
    Reg[ADS1115_REG_CONFIG] |= channel_config; // Set new MUX bits
    status = writeRegisters(ADS1115_REG_CONFIG);
    return status; 
}

/**
 * @brief Set the sample rate for conversions
 * @param rate Sample rate selection (8 to 860 samples per second)
 * @return esp_err_t ESP_OK on success, error code otherwise
 * @details Updates the DR bits in the configuration register to set conversion speed
 */
esp_err_t ADS1115::SetSampleRate(sSampleRate_t rate)
{
    esp_err_t status;
    status = ReadConfigReg();
    if ((status != ESP_OK))
    {
        return status;
    }
    Reg[ADS1115_REG_CONFIG] &= ~0x00E0; // Clear DR bits
    Reg[ADS1115_REG_CONFIG] |= (rate << 5); // Set new DR bits
    status = writeRegisters(ADS1115_REG_CONFIG);
    return status; 
}

/**
 * @brief Start a single-shot conversion
 * @return esp_err_t ESP_OK on success, error code otherwise
 * @details Sets the OS bit in configuration register to start one conversion
 */
esp_err_t ADS1115::StartSSConv()
{
    esp_err_t status;
    status = ReadConfigReg();
    if ((status != ESP_OK))
    {
        return status;
    }
    Reg[ADS1115_REG_CONFIG] |= ADS1115_OS_MASK; // Set OS bit to start single conversion
    status = writeRegisters(ADS1115_REG_CONFIG);
    return status; 
}

/**
 * @brief Set device to single-shot conversion mode
 * @return esp_err_t ESP_OK on success, error code otherwise
 * @details Sets the MODE bit in configuration register for single-shot operation
 */
esp_err_t ADS1115::SetSSMode()
{
    esp_err_t status;
    status = ReadConfigReg();
    if ((status != ESP_OK))
    {
        return status;
    }
    
    Reg[ADS1115_REG_CONFIG] |= ADS1115_MODE_SINGLESHOT_MASK; // Set to single-shot mode
    status = writeRegisters(ADS1115_REG_CONFIG);
    return status; 
}

/**
 * @brief Set comparator threshold values
 * @param lo_thresh Low threshold value
 * @param hi_thresh High threshold value
 * @return esp_err_t ESP_OK on success, error code otherwise
 * @details Sets the low and high threshold registers for comparator operation
 */
esp_err_t ADS1115::SetThresholds(uint16_t lo_thresh, uint16_t hi_thresh)
{
    esp_err_t status;
    Reg[ADS1115_REG_LO_THRESH] = lo_thresh;
    status = writeRegisters(ADS1115_REG_LO_THRESH);
    if ((status != ESP_OK))
    {
        return status;
    }
    Reg[ADS1115_REG_HI_THRESH] = hi_thresh;
    status = writeRegisters(ADS1115_REG_HI_THRESH);
    return status; 
}

/**
 * @brief Initialize comparator with specified configuration
 * @param mode Comparator mode (traditional or window)
 * @param pol Comparator polarity (active low or high)
 * @param lat Latching mode enable/disable
 * @param que Number of conversions before alert is asserted
 * @return esp_err_t ESP_OK on success, error code otherwise
 * @details Configures all comparator-related bits in configuration register
 */
esp_err_t ADS1115::Comp_Init(uint16_t mode, uint16_t pol, uint16_t lat, uint16_t que)
{
    esp_err_t status;
    status = ReadConfigReg();
    if ((status != ESP_OK))
    {
        return status;
    }
    Reg[ADS1115_REG_CONFIG] &= ~0x0007; // Clear COMP bits
    Reg[ADS1115_REG_CONFIG] |= (mode | pol | lat | que); // Set new COMP bits
    status = writeRegisters(ADS1115_REG_CONFIG);
    return status; 
}

/**
 * @brief Set comparator mode
 * @param mode Comparator mode (traditional or window)
 * @return esp_err_t ESP_OK on success, error code otherwise
 * @details Updates the COMP_MODE bit in configuration register
 */
esp_err_t ADS1115::Comp_SetMode(uint16_t mode)
{
    esp_err_t status;
    status = ReadConfigReg();
    if ((status != ESP_OK))
    {
        return status;
    }
    Reg[ADS1115_REG_CONFIG] &= ~0x0001; // Clear COMP_MODE bit
    Reg[ADS1115_REG_CONFIG] |= mode; // Set new COMP_MODE bit
    status = writeRegisters(ADS1115_REG_CONFIG);
    return status; 
}

/**
 * @brief Set comparator polarity
 * @param pol Comparator polarity (active low or high)
 * @return esp_err_t ESP_OK on success, error code otherwise
 * @details Updates the COMP_POL bit in configuration register
 */
esp_err_t ADS1115::Comp_SetPol(uint16_t pol)
{
    esp_err_t status;
    status = ReadConfigReg();
    if ((status != ESP_OK))
    {
        return status;
    }
    Reg[ADS1115_REG_CONFIG] &= ~0x0002; // Clear COMP_POL bit
    Reg[ADS1115_REG_CONFIG] |= pol; // Set new COMP_POL bit
    status = writeRegisters(ADS1115_REG_CONFIG);
    return status; 
}
/**
 * @brief Set comparator latching mode
 * @param lat Latching mode enable/disable
 * @return esp_err_t ESP_OK on success, error code otherwise
 * @details Updates the COMP_LAT bit in configuration register
 */
esp_err_t ADS1115::Comp_SetLat(uint16_t lat)
{
    esp_err_t status;
    status = ReadConfigReg();
    if ((status != ESP_OK))
    {
        return status;
    }
    Reg[ADS1115_REG_CONFIG] &= ~0x0004; // Clear COMP_LAT bit
    Reg[ADS1115_REG_CONFIG] |= lat; // Set new COMP_LAT bit
    status = writeRegisters(ADS1115_REG_CONFIG);
    return status; 
}

/**
 * @brief Set comparator queue and disable
 * @param que Number of conversions before alert is asserted
 * @return esp_err_t ESP_OK on success, error code otherwise
 * @details Updates the COMP_QUE bits in configuration register
 */
esp_err_t ADS1115::Comp_SetQue(uint16_t que)
{
    esp_err_t status;
    status = ReadConfigReg();
    if ((status != ESP_OK))
    {
        return status;
    }
    Reg[ADS1115_REG_CONFIG] &= ~0x0003; // Clear COMP_QUE bits
    Reg[ADS1115_REG_CONFIG] |= que; // Set new COMP_QUE bits
    status = writeRegisters(ADS1115_REG_CONFIG);
    return status; 
}

