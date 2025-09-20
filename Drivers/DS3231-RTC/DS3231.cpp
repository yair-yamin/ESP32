#include "DS3231.h"
/**
 ******************************************************************************
 * @file    DS3231.c
 * @author  Yair Yamin
 * @date    17-09-2025
 * @brief   Driver for the DS3231 Real-Time Clock (RTC) module.
 * @see     https://www.analog.com/media/en/technical-documentation/data-sheets/ds3231.pdf
 * @details This driver, designed for STM32 microcontrollers using the WIRE
 * library, provides a comprehensive set of functions to manage the
 * time, date, alarms, and other features of the DS3231 RTC.
 *
 * Key Features:
 * - Time and Date Management: Allows you to easily set and retrieve
 * the current time (hours, minutes, seconds), date (day, month,
 * year), and day of the week. It correctly handles the
 * Binary-Coded Decimal (BCD) format required by the DS3231.
 *
 * - Configurable Alarms: A standout feature is its support for both
 * Alarm 1 and Alarm 2. You can configure these alarms with various
 * trigger modes, including once per second, or matching by minute,
 * hour, date, or day.
 *
 * - Square Wave/PWM Output: The driver includes a function to
 * configure the SQW pin. This allows the DS3231 to output a
 * square wave at one of several fixed frequencies (1Hz, 1.024kHz,
 * 4.096kHz, or 8.192kHz).
 *
 * - Temperature Sensor: It also includes a function to read the
 * on-chip temperature sensor, providing ambient temperature
 * readings with a resolution of 0.25°C.
 ******************************************************************************
 */

/* =============================== Global Variables =============================== */

/* ========================== Function Definitions ============================ */

/**
 * @brief Constructor for the DS3231 RTC module
 * @param handle Pointer to TwoWire I2C interface
 * @param addr I2C address of the DS3231 device
 * @param time Initial time structure containing hours, minutes, and seconds
 * @param date Initial date structure containing date, month, and year
 * @param dow Initial day of the week
 * @note Initializes the I2C interface and allocates memory for registers
 */
DS3231::DS3231(TwoWire* handle,uint8_t addr,const ds3231_time_t time,const ds3231_date_t date,const DOW_t dow):
 i2c_handle(handle),i2c_address(addr), time(time),date(date),dayOfWeek(dow) 
{
    // Initialize all registers to 0
    memset(Reg, 0, sizeof(Reg));
};

DS3231::~DS3231()
{}

/**
 * @brief Initializes the DS3231 RTC module
 * @return ESP_OK on success, ESP_ERR_* on failure
 * @details Initializes the device by setting the time, date, and day of week
 *         stored in the class members. Each operation is validated using
 *         the VALID macro.
 */
esp_err_t DS3231::Init()
{
    VALID(SetTime());
    VALID(SetDate());
    VALID(SetDOW());
    return ESP_OK;
}

/**
 * @brief Sets the date in the DS3231 RTC
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if date values are invalid
 * @details Validates and sets the date using the internal date structure:
 *         - Date must be 1-31 (depending on month)
 *         - Month must be 1-12
 *         - Year must be 0-99 (represents years 2000-2099)
 *         Date values are converted to BCD format before being written
 *         to the appropriate registers. Note: This function does not
 *         validate the number of days in each month.
 */
esp_err_t DS3231::SetDate(void)
{
    // Validate the input date values
    if(date.date > 31 || date.month < 1 || date.month > 12 || date.year > 99) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Convert BCD format
    Reg[DS3231_REG_DATE] = decToBCD(date.date);
    Reg[DS3231_REG_MONTH] =  decToBCD(date.month);
    Reg[DS3231_REG_YEAR] =decToBCD(date.year);

    return writeRegisters(DS3231_REG_DATE,3);
}

/**
 * @brief Retrieves the current date from the DS3231 RTC
 * @return ds3231_date_t structure containing date, month, and year
 * @details Reads three consecutive registers starting from DS3231_REG_DATE
 *         and converts the BCD values to decimal format. If the read operation
 *         fails, returns the last known date values.
 */
ds3231_date_t DS3231::GetDate()
{   
    esp_err_t status;
    status = readRegisters(DS3231_REG_DATE,3);
    if (status == ESP_OK) {
        date.date = BCDToDec(Reg[DS3231_REG_DATE]);
        date.month = BCDToDec(Reg[DS3231_REG_MONTH]);
        date.year = BCDToDec(Reg[DS3231_REG_YEAR]);
    }
    return date;
}

/**
 * @brief Sets the time in the DS3231 RTC
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if time values are invalid
 * @details Validates and sets the time using the internal time structure:
 *         - Hours must be 0-23 (24-hour format)
 *         - Minutes must be 0-59
 *         - Seconds must be 0-59
 *         Time values are converted to BCD format before being written
 *         to the appropriate registers.
 */
esp_err_t DS3231::SetTime()
{
    // Validate the input time values
    if(time.hours > 23 || time.minutes > 59 || time.seconds > 59) {
        return ESP_ERR_INVALID_ARG ;
    }
    
    // Convert BCD format
    Reg[DS3231_REG_SECONDS] = decToBCD(time.seconds);
    Reg[DS3231_REG_MINUTES] = decToBCD(time.minutes);
    Reg[DS3231_REG_HOURS] = decToBCD(time.hours);

    return writeRegisters(DS3231_REG_SECONDS,3);
}

/**
 * @brief Retrieves the current time from the DS3231 RTC
 * @return ds3231_time_t structure containing hours, minutes, and seconds
 * @details Reads three consecutive registers starting from DS3231_REG_SECONDS
 *         and converts the BCD values to decimal format. If the read operation
 *         fails, returns the last known time values.
 */
ds3231_time_t DS3231::GetTime()
{
    esp_err_t status;

    status = readRegisters(DS3231_REG_SECONDS,3);
    if (status == ESP_OK) {
        time.seconds = BCDToDec(Reg[DS3231_REG_SECONDS]);
        time.minutes = BCDToDec(Reg[DS3231_REG_MINUTES]);
        time.hours = BCDToDec(Reg[DS3231_REG_HOURS]);
    }
    return time;
}

/**
 * @brief Sets the day of week in the DS3231 RTC
 * @return ESP_OK on success, ESP_ERR_* on failure
 * @details Sets the day of week using the internal dayOfWeek value.
 *         The day of week is stored as a value from 1-7, where:
 *         1 = Sunday, 2 = Monday, ..., 7 = Saturday
 *         No validation is performed as the DOW_t enum ensures valid values.
 */
esp_err_t DS3231::SetDOW(void)
{
    Reg[DS3231_REG_DAY] =dayOfWeek;
    return writeRegisters(DS3231_REG_DAY,1);
}

/**
 * @brief Retrieves the current day of week from the DS3231 RTC
 * @return DOW_t enumeration value representing the day of week (Sunday-Saturday)
 * @details Reads the DS3231_REG_DAY register and converts it to the DOW_t
 *         enumeration type. If the read operation fails, returns the last
 *         known day of week value.
 */
DOW_t DS3231::GetDOW(void)
{
    esp_err_t status;
    DOW_t *dow = &dayOfWeek;
    status = readRegisters(DS3231_REG_DAY,1);
    if (status == ESP_OK) {
        *dow = (DOW_t)(Reg[DS3231_REG_DAY]);
    }
    return (*dow);
}

/**
 * @brief Sets Alarm 1 with specified mode and alarm settings
 * @param mode Operating mode for the alarm (EveryMinute, EveryHour, EveryDay, Once, EveryWeek)
 * @param alr Alarm settings structure containing time and date/day information
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if parameters are invalid
 * @details Configures Alarm 1 with the following mode options:
 *         - EveryMinute: Triggers every minute when seconds match
 *         - EveryHour: Triggers every hour when minutes and seconds match
 *         - EveryDay: Triggers every day when hours, minutes, and seconds match
 *         - Once: Triggers once when date and time match exactly
 *         - EveryWeek: Triggers weekly when day and time match
 *         The function also enables the Alarm 1 interrupt and clears any existing alarm flags
 */
esp_err_t DS3231::SetAlarm1(sMode_t mode,const sAlram_t alr)
{
    uint8_t A1M1, A1M2, A1M3, A1M4;
    uint8_t temp = 0;
    esp_err_t status;
    sAlram_t *alarm = &alarm1;

    // Validate the input alarm values
    if(alr.hours > 23 || alr.minutes > 59 || alr.seconds > 59 || (alr.dayOfWeek < Sunday || alr.dayOfWeek > Saturday) || alr.date < 1 || alr.date > 31) {
        return ESP_ERR_INVALID_ARG;
    }
    (*alarm) = alr ;

    switch (mode)
    {

    case EveryMinute:
        A1M1 = 0b00000000;
        A1M2 = 0b10000000;  
        A1M3 = 0b10000000;
        A1M4 = 0b10000000;

        Reg[DS3231_REG_ALARM1_DAYDATE] = A1M4 ;

    break;
    case EveryHour:
        A1M1 = 0b00000000;
        A1M2 = 0b00000000;
        A1M3 = 0b10000000;
        A1M4 = 0b10000000;
        Reg[DS3231_REG_ALARM1_DAYDATE] = A1M4 ;
    break;
    case EveryDay:
        A1M1 = 0b00000000;
        A1M2 = 0b00000000;  
        A1M3 = 0b00000000;
        A1M4 = 0b10000000;

        Reg[DS3231_REG_ALARM1_DAYDATE] = A1M4 ;
    break;
    case Once:
        A1M1 = 0b00000000;
        A1M2 = 0b00000000;
        A1M3 = 0b00000000;
        A1M4 = 0b00000000;
        temp = (alarm->date / 10) << 4;
        Reg[DS3231_REG_ALARM1_DAYDATE] = A1M4 | temp | (alarm->date % 10); // Set the alarm to trigger on a specific date
    break;
    case EveryWeek:
        A1M1 = 0b00000000;
        A1M2 = 0b00000000;  
        A1M3 = 0b00000000;
        A1M4 = 0b00000000;
        Reg[DS3231_REG_ALARM1_DAYDATE] = A1M4 | ALRAM_DAY_MASK | alarm->dayOfWeek; //Set the Alram to trigger on a specific day of the week
    break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    // Convert BCD format
    Reg[DS3231_REG_ALARM1_SECONDS] = A1M1 | decToBCD(alarm->seconds);
    Reg[DS3231_REG_ALARM1_MINUTES] = A1M2 | decToBCD(alarm->minutes);
    Reg[DS3231_REG_ALARM1_HOURS] = A1M3 | decToBCD(alarm->hours);

    // Clear the A1F (Alarm 1 Flag) before setting the alarm
    CLearAlarmsFlags();
    
    status = writeRegisters(DS3231_REG_ALARM1_SECONDS,4);
    if (status != ESP_OK) {
        return status;
    }

    Reg[DS3231_REG_CONTROL] =  INTR_MODE_MASK |ALARM1_MASK ; // Enable Alarm 1 Interrupt

    status = writeRegisters(DS3231_REG_CONTROL,1);
    if (status != ESP_OK) {
        return status;
    }
    return status;
}

/**
 * @brief Retrieves the current settings of Alarm 1
 * @return sAlram_t structure containing alarm settings (seconds, minutes, hours, day/date)
 * @details Reads four consecutive registers starting from DS3231_REG_ALARM1_SECONDS
 *         and converts the BCD values to decimal format. The alarm can be set to
 *         trigger on a specific time and either a specific date or day of week.
 *         If the read operation fails, returns the last known alarm values.
 */
sAlram_t DS3231::GetAlarm1()
{
    esp_err_t status;
    sAlram_t *alarm = &alarm1;
    status = readRegisters(DS3231_REG_ALARM1_SECONDS,4);
    if (status == ESP_OK) {
        alarm->seconds = (Reg[DS3231_REG_ALARM1_SECONDS] & 0x0F) + ((Reg[DS3231_REG_ALARM1_SECONDS] >> 4) * 10);
        alarm->minutes = (Reg[DS3231_REG_ALARM1_MINUTES] & 0x0F) + ((Reg[DS3231_REG_ALARM1_MINUTES] >> 4) * 10);
        alarm->hours   = (Reg[DS3231_REG_ALARM1_HOURS] & 0x0F) + ((Reg[DS3231_REG_ALARM1_HOURS] >> 4) * 10);
        alarm->dayOfWeek = (DOW_t)(Reg[DS3231_REG_ALARM1_DAYDATE]);
        alarm->date = (Reg[DS3231_REG_ALARM1_DAYDATE] & 0x0F) + ((Reg[DS3231_REG_ALARM1_DAYDATE] >> 4) * 10);
    }
    return (*alarm);
}

/**
 * @brief Sets Alarm 2 with specified mode and alarm settings
 * @param mode Operating mode for the alarm (EveryMinute, EveryHour, EveryDay, Once, EveryWeek)
 * @param alr Alarm settings structure containing time and date/day information
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if parameters are invalid
 * @details Configures Alarm 2 with the following mode options:
 *         - EveryMinute: Triggers every minute (no seconds matching as Alarm 2 lacks seconds register)
 *         - EveryHour: Triggers every hour when minutes match
 *         - EveryDay: Triggers every day when hours and minutes match
 *         - Once: Triggers once when date and time match exactly
 *         - EveryWeek: Triggers weekly when day and time match
 *         The function also enables the Alarm 2 interrupt and clears any existing alarm flags.
 *         Note: Alarm 2 does not have seconds capability.
 */
esp_err_t DS3231::SetAlarm2(sMode_t mode,const sAlram_t alr)
{
    uint8_t  A2M2, A2M3, A2M4;
    uint8_t temp = 0;
    esp_err_t status;
    sAlram_t *alarm = &alarm2;
    // Validate the input alarm values
    if(alr.hours > 23 || alr.minutes > 59 || (alr.dayOfWeek < Sunday || alr.dayOfWeek > Saturday) || alr.date < 1 || alr.date > 31) {
        return ESP_ERR_INVALID_ARG;
    }
    (*alarm) = alr ;
    switch (mode)
    {

    case EveryMinute:
       
        A2M2 = 0b10000000;
        A2M3 = 0b10000000;
        A2M4 = 0b10000000;
        Reg[DS3231_REG_ALARM2_DAYDATE] = A2M4;
    break;
    case EveryHour:
        A2M2 = 0b00000000;
        A2M3 = 0b10000000;
        A2M4 = 0b10000000;
        Reg[DS3231_REG_ALARM2_DAYDATE] = A2M4;
    break;
    case EveryDay:
        
        A2M2 = 0b00000000;
        A2M3 = 0b00000000;
        A2M4 = 0b10000000;
        Reg[DS3231_REG_ALARM2_DAYDATE] = A2M4;
    break;
    case Once:
       
        A2M2 = 0b00000000;
        A2M3 = 0b00000000;
        A2M4 = 0b00000000;
        temp = (alarm->date / 10) << 4;
        Reg[DS3231_REG_ALARM2_DAYDATE] = A2M4 | temp | (alarm->date % 10);
    break;
    case EveryWeek:
        
        A2M2 = 0b00000000;
        A2M3 = 0b00000000;
        A2M4 = 0b00000000;
        Reg[DS3231_REG_ALARM2_DAYDATE] = A2M4 | ALRAM_DAY_MASK | alarm->dayOfWeek; //Set the Alram to trigger on a specific day of the week
    break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    // Convert BCD format
    Reg[DS3231_REG_ALARM2_MINUTES] = A2M2 | decToBCD(alarm->minutes);
    Reg[DS3231_REG_ALARM2_HOURS] = A2M3 | decToBCD(alarm->hours) ;

    // Clear the A2F (Alarm 2 Flag) before setting the alarm
    CLearAlarmsFlags();
    status = writeRegisters(DS3231_REG_ALARM2_MINUTES,3);
    if (status != ESP_OK) {
        return status;
    }

    Reg[DS3231_REG_CONTROL] |= INTR_MODE_MASK | ALARM2_MASK; // Enable Alarm 2 & Interrupt mode
    status = writeRegisters(DS3231_REG_CONTROL,1);
    if (status != ESP_OK) {
        return status;
    }

   
    return status;
}

/**
 * @brief Retrieves the current settings of Alarm 2
 * @return sAlram_t structure containing alarm settings (seconds, minutes, hours, day/date)
 * @details Reads four consecutive registers starting from DS3231_REG_ALARM1_SECONDS
 *         and converts the BCD values to decimal format. The alarm can be set to
 *         trigger on a specific time and either a specific date or day of week.
 *         If the read operation fails, returns the last known alarm values.
 */
sAlram_t DS3231::GetAlarm2()
{
    esp_err_t status;
    sAlram_t *alarm = &alarm2;
    status = readRegisters(DS3231_REG_ALARM2_MINUTES,3);
    if (status == ESP_OK) {
        alarm->minutes = (Reg[DS3231_REG_ALARM2_MINUTES] & 0x0F) + ((Reg[DS3231_REG_ALARM2_MINUTES] >> 4) * 10);
        alarm->hours   = (Reg[DS3231_REG_ALARM2_HOURS] & 0x0F) + ((Reg[DS3231_REG_ALARM2_HOURS] >> 4) * 10);
        alarm->dayOfWeek = (DOW_t)(Reg[DS3231_REG_ALARM2_DAYDATE]);
        alarm->date = (Reg[DS3231_REG_ALARM2_DAYDATE] & 0x0F) + ((Reg[DS3231_REG_ALARM2_DAYDATE] >> 4) * 10);
        alarm->seconds = 0; // Alarm B does not use seconds, so set it to 0
    }
    return (*alarm);
}

/**
 * @brief Writes the current status register value to the DS3231
 * @return ESP_OK on success, ESP_ERR_NOT_FINISHED if write fails
 * @details Writes the value stored in the internal Reg buffer to the
 *         DS3231's status register. This function is typically used after
 *         modifying status register bits to:
 *         - Clear oscillator stop flag (OSF)
 *         - Enable/disable 32kHz output
 *         - Set temperature conversion rate
 *         - Clear alarm flags
 *         The status register value should be modified before calling this function.
 */
esp_err_t DS3231::WriteStatus(void)
{
    return writeRegisters(DS3231_REG_STATUS,1);
}

/**
 * @brief Reads the status register of the DS3231
 * @param[out] stat Pointer to store the status register value
 * @return ESP_OK on success, ESP_ERR_* on failure
 * @details The status register contains:
 *         - OSF: Oscillator Stop Flag
 *         - BB32KHZ: Battery Backed 32kHz output
 *         - CRATE: Conversion rate
 *         - EN32KHZ: Enable 32kHz Output
 *         - BSY: Device Busy Flag
 *         - A2F: Alarm 2 Flag
 *         - A1F: Alarm 1 Flag
 */
esp_err_t DS3231::ReadStatus(uint8_t * stat)
{
    esp_err_t status;
    status = readRegisters(DS3231_REG_STATUS,1);
    
    if(status == ESP_OK)
    {
        (*stat) = Reg[DS3231_REG_STATUS];
    }

    return status;
}

/**
 * @brief Reads the temperature from the DS3231's built-in temperature sensor
 * @return Float value representing the temperature in degrees Celsius
 * @details Reads both MSB and LSB temperature registers and combines them to
 *         create a 10-bit temperature value. The raw value is then converted
 *         to degrees Celsius with a resolution of 0.25°C. If the read
 *         operation fails, returns the last known temperature value.
 */
float DS3231::GetTemp()
{
    esp_err_t status;
    float *temperature = &temp;
    int16_t raw_value = 0;
    status = readRegisters(DS3231_REG_TEMP_MSB,2);
    if (status == ESP_OK) {
        raw_value = (int16_t)(Reg[DS3231_REG_TEMP_MSB] << 2) | (Reg[DS3231_REG_TEMP_LSB] >> 6);
        *temperature = raw_value / 4.0f;
    }
    return (*temperature);
}

/**
 * @brief Reads the control register of the DS3231
 * @param[out] ctl Pointer to store the control register value
 * @return ESP_OK on success, ESP_ERR_* on failure
 * @details The control register contains settings for:
 *         - EOSC: Enable oscillator
 *         - BBSQW: Battery-backed square wave
 *         - CONV: Convert temperature
 *         - RS2/RS1: Rate select for square wave
 *         - INTCN: Interrupt control
 *         - A2IE/A1IE: Alarm interrupt enables
 */
esp_err_t DS3231::GetControlRegister(uint8_t *ctl)  
{
    esp_err_t status;
    status = readRegisters(DS3231_REG_CONTROL,1);
    
    if(status == ESP_OK)
    {
        (*ctl) = Reg[DS3231_REG_CONTROL];
    }

    return status;
}

/**
 * @brief Configures the square wave output on the SQW pin
 * @param RS2 Rate Select 2 bit value (0 or 1)
 * @param RS1 Rate Select 1 bit value (0 or 1)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if invalid parameters
 * @details Configures the frequency of the square wave output:
 *         RS2  RS1  Frequency
 *          0    0     1 Hz
 *          0    1   1.024 kHz
 *          1    0   4.096 kHz
 *          1    1   8.192 kHz
 */
esp_err_t DS3231::OutputPWM(uint8_t RS2, uint8_t RS1)
{
    // Validate the input values
    if(RS1 != 0 && RS1 != 1 && RS2 != 0 && RS2 != 1) {
        return ESP_ERR_INVALID_ARG;
    }

    // Set the control register for PWM output
    Reg[DS3231_REG_CONTROL] &= ~(PWM_MODE_MASK); // Clear INTCN bit
    Reg[DS3231_REG_CONTROL] = (RS1 << 3) | (RS2 << 4); // Set RS1 and RS2 bits

    return writeRegisters(DS3231_REG_CONTROL,1);
}

/**
 * @brief Writes multiple registers to the DS3231 via I2C
 * @param startReg Starting register address to write to
 * @param len Number of bytes to write
 * @return ESP_OK on successful write, ESP_ERR_NOT_FINISHED if write fails
 * @details Performs an I2C write operation by:
 *          1. Starting transmission to device address
 *          2. Writing the starting register address
 *          3. Writing len bytes from the internal Reg buffer
 *          4. Ending transmission with STOP condition
 */
esp_err_t DS3231::writeRegisters(uint8_t startReg, uint8_t len) {
  i2c_handle->beginTransmission(this->i2c_address);
  i2c_handle->write(startReg); // set the register pointer

  for (uint8_t i = 0; i < len; i++) {
    i2c_handle->write(Reg[startReg + i]); // write each byte
  }

  uint8_t result = i2c_handle->endTransmission(); // 0 = success

  if (result == 0) {
    return ESP_OK;
  } 
  return ESP_ERR_NOT_FINISHED;
}

/**
 * @brief Reads multiple registers from the DS3231 via I2C
 * @param reg Starting register address to read from
 * @param len Number of bytes to read
 * @return ESP_OK on successful read, ESP_ERR_NOT_FINISHED if read fails
 * @details Performs an I2C read operation following this sequence:
 *          1. Send device address with write bit
 *          2. Send register address to read from
 *          3. Send repeated start
 *          4. Send device address with read bit
 *          5. Read requested number of bytes
 *          The read bytes are stored in the internal Reg buffer
 */
esp_err_t DS3231::readRegisters(uint8_t reg,uint8_t len)
{
    esp_err_t status ;
    uint8_t bytesReceived = 0 ;
    // Start I2C communication with the DS3231
    i2c_handle->beginTransmission(this->i2c_address);
    // Specify which register we want to read from
    i2c_handle->write(reg);
    // Send restart condition - keep connection active without STOP
    i2c_handle->endTransmission(false);

    // Request 'len' bytes from the specified device address
    bytesReceived = i2c_handle->requestFrom(this->i2c_address, len);

    // Check if bytes are available and we received the expected amount
    if (i2c_handle->available() && bytesReceived == len) {
        // Read the bytes into our register buffer starting at specified register
        i2c_handle->readBytes(&this->Reg[reg], len);
        // Return success status
        return ESP_OK;
    }
    return ESP_ERR_NOT_FINISHED; 
}

/**
 * @brief Clears both alarm flag bits in the status register
 * @return ESP_OK on success, ESP_ERR_NOT_FINISHED if operation fails
 * @details This function should be called:
 *         1. After an alarm has triggered to reset the interrupt
 *         2. Before setting up a new alarm to ensure clean state
 *         3. When disabling alarms
 *         The function clears both A1F (Alarm 1 Flag) and A2F (Alarm 2 Flag)
 */
esp_err_t DS3231::CLearAlarmsFlags(void)
{
    uint8_t *st = &Reg[DS3231_REG_STATUS];
        if (readRegisters(DS3231_REG_STATUS,1) == ESP_OK) {  
            *st &= ~(1u<<0);  // clear A1F
            *st &= ~(1u<<1);  // clear A2F
            return writeRegisters(DS3231_REG_STATUS,1);
        }
    return ESP_ERR_NOT_FINISHED;
}

