/*
 * Copyright (c) 2017, CATIE, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "bme280.h"

namespace sixtron {
#define SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##__MSK)) |\
     (data & bitname##__MSK))

#define GET_BITS_POS_0(reg_data, bitname) \
    (reg_data & (bitname##__MSK))

/**\name MACROS DEFINITIONS                      */
#define INIT_VALUE                         0
#define SUCCESS                            0
#define FAILURE                            1
#define TEMP_PRESS_CALIB_DATA_LEN          26
/** Bit value manipulation                       */
#define ZERO                               0
#define ONE                                1
#define ONE_BIT_SHIFT                      1        
#define TWO_BITS_SHIFT                     2        
#define FOUR_BITS_SHIFT                    4
#define EIGHT_BITS_SHIFT                   8        
#define SIXTEEN_BITS_SHIFT                 16

/** Temp/Press/Humidity minimum/maximum values   */
#define TEMPERATURE_MIN                    -40
#define TEMPERATURE_MAX                    85
#define PRESSURE_MIN                       30000
#define PRESSURE_MAX                       110000
#define HUMIDITY_MIN                       100
#define HUMIDITY_MAX                       0

#define CONTROL_MEAS__MSK                  0xFC

#define SENSOR_MODE__MSK                   0x03
#define SENSOR_MODE__POS                   0x00

#define UNCOMPENSATED_TEMPERATURE__MSK     0xFFFFF
#define UNCOMPENSATED_PRESSURE_MSK         0xFFFFF

#define OSRS_T__POS                        0x5
#define OSRS_P__POS                        0x3
#define STANDBY__POS                       0x5
#define FILTER__POS                        0x3

#define SOFTRESET_CMD                      0xB6

/*!
 * @brief Default BME280 contructor
 *
 * @param i2c Instance of I2C
 * @param i2c_address I2C address of the device
 *
 */
BME280::BME280(I2C* i2c, I2CAddress i2c_address):
    _i2c(i2c), _i2c_address(i2c_address){
    _sensor_mode = SensorMode::NORMAL;
    t_fine = 0;
}

/*!
 * @brief Initialize the device
 *
 * @param None
 *
 * @return
 *         true on success,
 *         false on failure
 */
bool BME280::initialize(){
   printf("\nInitializing the BME280...\n");
   if (!read_chip_id())
       return false;
   else {
       printf("Chip ID: 0x%X\n", 0x60);
       _chip_id = 0x60;
   }

   if (reset() != SUCCESS)
       return false;

   // wait for chip to wake up
   wait_ms(1);

   get_calib();

   return true;
}

/*!
 * Perform a reset of the device
 * 
 * @param None
 *
 * @return
 *         0 on success,
 *         1 on failure
 */
int BME280::reset(){
    if (i2c_write_register(RegisterAddress::RESET, SOFTRESET_CMD) != SUCCESS)
        return FAILURE;
    wait_ms(2);
    return SUCCESS;
}

/*!
 * Read sensor measured humidity
 *
 * @param humidity Pointer to the value of the humidity, value in %RH
 *
 * @return 
 *         0 on success,
 *         1 on failure 
 */
int BME280::read_humidity(double* humidity){
    if (!humidity)
        return FAILURE;

    get_raw_data();

#if 0
    printf("raw humidity: %d\n", uncomp_data.humidity);
    printf("H calib: %u %d %d %d %d %d\n", 
            calib.dig_H1, calib.dig_H2, calib.dig_H3,
            calib.dig_H4, calib.dig_H5, calib.dig_H5);
#endif

    double var1, var2, var3, var4, var5, var6;
    var1 = t_fine - 76800.0;
    var2 = (static_cast<double>(calib.dig_H4) * 64.0) + ((static_cast<double>(calib.dig_H5) / 16384.0) * var1);
    var3 = static_cast<double>(uncomp_data.humidity) - var2;
    var4 = (static_cast<double>(calib.dig_H2) / 65536.0);
    var5 = 1.0 + ((static_cast<double>(calib.dig_H3) / 67108864.0) * var1);
    var6 = 1.0 + ((static_cast<double>(calib.dig_H6) / 67108864.0) * var1 * var5);
    var6 = var3 * var4 * (var5 * var6);
    *humidity = var6 * (1.0 - ((static_cast<double>(calib.dig_H1) * var6) / 524288.0));
    if (*humidity > HUMIDITY_MAX)
        *humidity = HUMIDITY_MAX;
    if (*humidity < HUMIDITY_MIN)
        *humidity = HUMIDITY_MIN;
    return SUCCESS;
}


/*!
 * @brief Read sensor pressure
 *
 * @param pressure Pointer to the value of the pressure, value in Pa 
 *
 * @return
 *        0 on success,
 *        1 on failure
 */
int BME280::read_pressure(double* pressure){
    if (!pressure)
        return FAILURE;

    get_raw_data();

    // Pressure may have been skipped
    if (uncomp_data.pressure == 0x80000)
        return FAILURE;

#if 0
    printf("raw pressure: %d\n", uncomp_data.pressure);
    printf("P calib: %u %d %d %d %d %d %d %d %d\n", 
            calib.dig_P1, calib.dig_P2, calib.dig_P3, calib.dig_P4,
            calib.dig_P5, calib.dig_P6, calib.dig_P7, calib.dig_P8, calib.dig_P9);
#endif

    double var1, var2, var3;
    int32_t raw_press = uncomp_data.pressure;

    var1 = (static_cast<double>(t_fine) / 2.0) - 64000.0;
    var2 = var1 * var1 * (static_cast<double>(calib.dig_P6)) / 32768.0;
    var2 = var2 + var1 * (static_cast<double>(calib.dig_P5)) * 2.0;
    var2 = (var2 / 4.0) + (static_cast<double>(calib.dig_P4)) * 65536.0;
    var3 = static_cast<double>(calib.dig_P3) * var1 * var1 / 524288.0;
    var1 = (var3 + static_cast<double>(calib.dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * (static_cast<double>(calib.dig_P1));
    /* avoid potential exception caused by division by zero */
    if (!var1)
        return FAILURE;

    *pressure = 1048576.0 - static_cast<double>(raw_press);
    *pressure = (*pressure - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = (static_cast<double>(calib.dig_P9)) * (*pressure) * (*pressure) / 2147483648.0;
    var2 = (*pressure) * (var1 + var2 + static_cast<double>(calib.dig_P7)) / 16.0;
    if (*pressure > PRESSURE_MAX)
        *pressure = PRESSURE_MAX;
    if (*pressure < PRESSURE_MIN)
        *pressure = PRESSURE_MIN;
    return SUCCESS;
}

/*!
 * @brief Read sensor temperature
 *
 * @param temperature Pointer to the value of the temperature in °C
 *
 * @return
 *         0 on success,
 *         1 on failure
 */
int BME280::read_temperature(double* temperature){
    if (!temperature)
        return FAILURE;

    get_raw_data();

    // Temperature may have been skipped
    if (uncomp_data.temperature == 0x80000)
    	return FAILURE;

#if 1 // debug
    printf("Temp raw: %d\n", uncomp_data.temperature);
    printf("t: Calib %u %d %d\n", calib.dig_T1, calib.dig_T2, calib.dig_T3);
#endif

    int32_t temp =
        (((((uncomp_data.temperature >> 3) - (calib.dig_T1 << 1))) * calib.dig_T2) >> 11) +
        ((((((uncomp_data.temperature >> 4) - calib.dig_T1) * ((uncomp_data.temperature >> 4) - calib.dig_T1)) >> 12) * calib.dig_T3) >> 14);

    t_fine = static_cast<double>(temp);
    temp = (temp * 5 + 128) >> 8;
    *temperature = (static_cast<double>(temp) / 100.0);
    if (*temperature > TEMPERATURE_MAX)
        *temperature = TEMPERATURE_MAX;
    if (*temperature < TEMPERATURE_MIN)
        *temperature = TEMPERATURE_MIN;

    return SUCCESS;
}

/*!
 * @brief Read sensor environmental parameters
 *
 * @param env Pointer to the structure containing the values of temperature,
 * pressure and humidity.
 *
 * @return
 *         0 on success,
 *         1 on failure
 */
int BME280::read_env_data(bme280_environment_t* env){
    if (!env)
    	return FAILURE;
    if (read_temperature(&env->temperature) != SUCCESS)
    	return FAILURE;
    if (read_pressure(&env->pressure) != SUCCESS)
    	return FAILURE;
    if (read_humidity(&env->humidity) != SUCCESS)
    	return FAILURE;
    return SUCCESS;
}

/*!
 *
 * TODO
 */
int BME280::write_power_mode(SensorMode mode){
    int8_t sensor_mode;
    if (i2c_read_register(RegisterAddress::CONTROL_MEAS, &sensor_mode) != SUCCESS)
        return FAILURE;
    sensor_mode = SET_BITS_POS_0(sensor_mode, SENSOR_MODE, static_cast<int8_t>(mode));
    if (i2c_write_register(RegisterAddress::CONTROL_MEAS, sensor_mode) != SUCCESS)
        return FAILURE;
    return SUCCESS;
}

/*!
 *
 * TODO
 */
int BME280::sleep(){
    static char data[4];
    data[0] = static_cast<char>(RegisterAddress::CONTROL_HUMID);
    if (_i2c->write(static_cast<int>(_i2c_address) << 1, data, 1, true) != SUCCESS)
        return FAILURE;
    if (_i2c->read(static_cast<int>(_i2c_address) << 1, data, 4, false) != SUCCESS)
        return FAILURE;
    return reset();
}

/*!
 *
 * TODO
 */
void BME280::take_forced_measurement(){
	if (_sensor_mode == SensorMode::FORCED){
		if (i2c_write_register(RegisterAddress::CONTROL_MEAS, 
                    static_cast<int8_t>((settings.osrs_t << OSRS_T__POS) 
                        | (settings.osrs_p << OSRS_P__POS) 
                        | static_cast<char>(_sensor_mode))) != SUCCESS)
			return;
		int8_t data;
		while (true){
			if (i2c_read_register(RegisterAddress::STATUS, &data) != SUCCESS)
				return;
			if (!(data & 0x08)) // STATUS measuring[0] bit (0 when the results have been transferred to the data registers)
				break;
			wait_ms(1);
		}
	}
}

/*!
 * @brief Set the device power mode
 *
 * @param mode Chosen power mode
 *
 * value  | power mode
 * ------------------
 *   0b00 | SLEEP
 *   0b01 | FORCED
 *   0b10 | NORMAL
 *
 * @return
 *         0 on success,
 *         1 on failure
 */
int BME280::set_power_mode(SensorMode mode){
    SensorMode last_set_mode;
    if (get_power_mode(&last_set_mode) != SUCCESS)
        return FAILURE;
    if (last_set_mode != SensorMode::SLEEP){
        if (sleep() != SUCCESS)
            return FAILURE;
        write_power_mode(mode);    
        return SUCCESS;
    }
    return FAILURE;
}

/*!
 * @brief Get the device power mode
 *
 * @param mode Pointer to the value of power mode
 *
 * value | power mode
 * ------------------
 *   0   | SLEEP
 *   1   | FORCED
 *   2   | NORMAL
 *
 * @return 
 *        0 on success,
 *        1 on failure
 */
int BME280::get_power_mode(SensorMode* mode){
    if (mode){
        *mode = _sensor_mode;
        return SUCCESS;
    }
    return FAILURE;
}


/*!
 * @brief Set sampling settings
 *
 * @param mode Sensor mode to set 
 * @param temp_sampling Temperature sampling to set
 * @param press_sampling Pressure sampling to set
 * @param humid_sampling Humidity sampling to set
 * @param filter Filter setting to set
 * @param duration Stand-by duration
 *
 * @return None
 */
void BME280::set_sampling(SensorMode mode, 
        SensorSampling temp_sampling, 
        SensorSampling press_sampling,
        SensorSampling humid_sampling,
        SensorFilter filter,
        StandbyDuration duration){
    _sensor_mode = mode;
    settings.osrs_t = static_cast<uint8_t>(temp_sampling);
    settings.osrs_h = static_cast<uint8_t>(humid_sampling);
    settings.osrs_p = static_cast<uint8_t>(press_sampling);
    settings.filter = static_cast<uint8_t>(filter);
    settings.standby_time = static_cast<uint8_t>(duration);

    if (i2c_write_register(RegisterAddress::CONTROL_HUMID, 
            static_cast<int8_t>(settings.osrs_h)) != SUCCESS)
        return;

    if (i2c_write_register(RegisterAddress::CONTROL_MEAS, 
            static_cast<int8_t>((settings.osrs_t << OSRS_T__POS) 
            | (settings.osrs_p << OSRS_P__POS) 
            |  static_cast<char>(mode))) != SUCCESS)
        return;

    if (i2c_write_register(RegisterAddress::CONFIG,
            static_cast<int8_t>((settings.standby_time << STANDBY__POS)
            | (settings.filter << FILTER__POS) | 0)) != SUCCESS)
        return;
}

/*!
 * @brief Private function to check chip ID correctness
 *
 * @param None
 *
 * @return
 *         true on success,
 *         false on failure
 */
bool BME280::read_chip_id(){
   int8_t chip_id = INIT_VALUE;
   if (i2c_read_register(RegisterAddress::CHIP_ID, &chip_id) != SUCCESS)
       return FAILURE;
   if (chip_id != 0x60){
       wait_ms(1000);
       i2c_read_register(RegisterAddress::CHIP_ID, &chip_id);
       return (chip_id != 0x60) ? false : true;
   }
   return true;
}

/*!
 * @brief Read and store calibration data
 *
 * @param None
 *
 * @return None
 */
void BME280::get_calib(){
    int8_t s8_dig_1, s8_dig_2;
    int8_t s8_dig[2];

    /* Temperature-related coefficients */
    static char data[18];
    data[0] = static_cast<char>(RegisterAddress::DIG_T1);
    _i2c->write(static_cast<int>(_i2c_address) << 1, data, 1, true);
    _i2c->read(static_cast<int>(_i2c_address) << 1, data, 6, false);
    calib.dig_T1 = static_cast<uint16_t>((data[1] << 8) | (data[0]));
    calib.dig_T2 = (data[3] << 8) | (data[2]);
    calib.dig_T3 = (data[5] << 8) | (data[4]);

#if 0
    printf("dig_T = %u %d %d\n", calib.dig_T1, calib.dig_T2, calib.dig_T3);
#endif

    /* Pressure-related coefficients */
    data[0] = static_cast<char>(RegisterAddress::DIG_P1);
    _i2c->write(static_cast<int>(_i2c_address) << 1, data, 1, true);
    _i2c->read(static_cast<int>(_i2c_address) << 1, data, 18, false);
    calib.dig_P1 = (data[ 1] << 8) | data[ 0];
    calib.dig_P2 = (data[ 3] << 8) | data[ 2];
    calib.dig_P3 = (data[ 5] << 8) | data[ 4];
    calib.dig_P4 = (data[ 7] << 8) | data[ 6];
    calib.dig_P5 = (data[ 9] << 8) | data[ 8];
    calib.dig_P6 = (data[11] << 8) | data[10];
    calib.dig_P7 = (data[13] << 8) | data[12];
    calib.dig_P8 = (data[15] << 8) | data[14];
    calib.dig_P9 = (data[17] << 8) | data[16];

#if 0
    printf("dig_P = %u %d %d %d %d %d %d %d %d\n", calib.dig_P1, calib.dig_P2, calib.dig_P3, calib.dig_P4, calib.dig_P5, calib.dig_P6, calib.dig_P7, calib.dig_P8, calib.dig_P9);
#endif

    /* Humidity-related coefficients */
    i2c_read_register(RegisterAddress::DIG_H1, &s8_dig_1);
    calib.dig_H1 = static_cast<uint8_t>(s8_dig_1);
    i2c_read_two_bytes(RegisterAddress::DIG_H2, s8_dig);
    calib.dig_H2 = (s8_dig[1] << 8 | s8_dig[0]);
    i2c_read_register(RegisterAddress::DIG_H3, &s8_dig_1);
    calib.dig_H3 = static_cast<uint8_t>(s8_dig_1);
    i2c_read_two_bytes(RegisterAddress::DIG_H4, s8_dig);
    calib.dig_H4 = (s8_dig[0] << 4 | (s8_dig[1] & 0xF));

    //FIXME

    i2c_read_register(RegisterAddress::DIG_H5, s8_dig);
    i2c_read_register(static_cast<RegisterAddress>(static_cast<char>(RegisterAddress::DIG_H5) + 1), &s8_dig_2);
    calib.dig_H5 = (s8_dig[1] << 4 | (s8_dig[0] & 0xF));
    i2c_read_register(RegisterAddress::DIG_H6, &calib.dig_H6);

}

/*!
 * @brief Parse raw data: pressure, humidity and temperature, then store it to
 * uncomp_data attribute
 *
 * @param None
 *
 * @return None
 */
void BME280::get_raw_data(){
    int8_t data_3[3];
    int8_t data_2[2];
    char cmd[4];
    
    // raw_pressure = PRESS_MSB | PRESS_LSB | PRESS_XLSB[7:4]
    cmd[0] = static_cast<char>(RegisterAddress::PRESS_MSB);
    _i2c->write((static_cast<int>(_i2c_address)) << 1, cmd, 1, true);
    _i2c->read((static_cast<int>(_i2c_address)) << 1, &cmd[1], 3, true);
    uncomp_data.pressure = static_cast<uint32_t>((cmd[1] << 12) | (cmd[2] << 4) | (cmd[3] >> 4));
    uncomp_data.pressure &= UNCOMPENSATED_PRESSURE_MSK;


    // raw_temperature = TEMP_MSB | TEMP_LSB | TEMP_XLSB[7:4]
    cmd[0] = static_cast<char>(RegisterAddress::TEMP_MSB);
    _i2c->write((static_cast<int>(_i2c_address)) << 1, cmd, 1, true);
    _i2c->read((static_cast<int>(_i2c_address)) << 1, &cmd[1], 3, true);
    uncomp_data.temperature = ((cmd[1] << 12) | (cmd[2] << 4) | (cmd[3] >> 4));
    uncomp_data.temperature &= UNCOMPENSATED_TEMPERATURE__MSK;

    // raw_humidity = HUMID_MSB | HUMID_LSB
    cmd[0] = static_cast<char>(RegisterAddress::HUMID_MSB);
    _i2c->write((static_cast<int>(_i2c_address)) << 1, cmd, 1, true);
    _i2c->read((static_cast<int>(_i2c_address)) << 1, &cmd[1], 2, true);
    uncomp_data.humidity = static_cast<int16_t>(cmd[1] << EIGHT_BITS_SHIFT |
            cmd[2]);
}

/*!
 * @brief Read register data
 *
 * @param register_address Address of the register
 * @param value Pointer to the value read from the register
 *
 * @return 
 *         0 on success,
 *         1 on failure
 */
int BME280::i2c_read_register(RegisterAddress register_address, int8_t* value){
    static char data;
    data = static_cast<char>(register_address);
    if (_i2c->write(static_cast<int>(_i2c_address) << 1, &data, 1, true) != SUCCESS)
		return FAILURE;
	char* char_value = reinterpret_cast<char* >(value);
    if (_i2c->read(static_cast<int>(_i2c_address) << 1, char_value, 1) != SUCCESS)
        return FAILURE;
    return SUCCESS;
}

/*!
 * @brief Read two successive registers data
 *
 * @note This function is useful to read memory-contiguous LSB/MSB registers
 *
 * @param register_address Address of the first register
 * @param value Array to store read data
 *
 * @return 
 *         0 on success,
 *         1 on failure
 */
int BME280::i2c_read_two_bytes(RegisterAddress register_address, int8_t value[2]){
    static char data[2];
    data[0] = static_cast<char>(register_address);
    if (_i2c->write(static_cast<int>(_i2c_address) << 1, data, 1, true) != SUCCESS)
        return FAILURE;
    if (_i2c->read(static_cast<int>(_i2c_address) << 1, data, 2, false) != SUCCESS)
        return FAILURE;
    for (int i = 0; i < 2; i++)
        value[i] = data[i];
    return SUCCESS;
}

/*!
 * @brief Read three successive registers data
 *
 * @note This function is useful to read memory-contiguous LSB/MSB/XLSB registers
 *
 * @param register_address Address of the first register
 * @param value Array to store the read data
 *
 * @return 
 *         0 on success,
 *         1 on failure
 */
int BME280::i2c_read_three_bytes(RegisterAddress register_address, int8_t value[3]){
    char data[3];
    data[0] = static_cast<char>(register_address);
    if (_i2c->write(static_cast<int>(_i2c_address) << 1, data, 1, true) != SUCCESS)
        return FAILURE;
    if (_i2c->read(static_cast<int>(_i2c_address) << 1, data, 3, false) != SUCCESS)
        return FAILURE;
    for (int i = 0; i < 3; i++)
        value[i] = data[i];
    return SUCCESS;
}

/*!
 * @brief Read a 16 bits signed vector (3 dimensions) continuous read
 *
 * @param register_address Address of the register containing the LSB 
 *                        of the first axis
 *        value Array to store the read values
 *
 * @return
 *         0 on success,
 *         1 on failure
 */
int BME280::i2c_read_vector(RegisterAddress register_address, int16_t value[3]){
    static char data[6];
    data[0] = static_cast<char>(register_address);
    if (_i2c->write(static_cast<int>(_i2c_address) << 1, data, 1, true) != SUCCESS)
        return FAILURE;
    if (_i2c->read(static_cast<int>(_i2c_address) << 1, data, 6, false) != SUCCESS)
        return FAILURE;
    for (int i = 0; i < 3; i++)
        value[i] = (data[2*i + 1] << EIGHT_BITS_SHIFT) | (data[2*i]);
    return SUCCESS;
}

/*!
 * @brief Write to a register
 *
 * @param register_address Address of the register to write to
 * @param value Data to store in the register
 *
 * @return 
 *         0 on success,
 *         1 on failure
 */
int BME280::i2c_write_register(RegisterAddress register_address, int8_t value){
    static char data[2];
    data[0] = static_cast<char>(register_address);
    data[1] = static_cast<char>(value);
    if (_i2c->write(static_cast<int>(_i2c_address) << 1, data, 2) != SUCCESS)
        return FAILURE;
    return SUCCESS;
}

} // namespace sixtron
