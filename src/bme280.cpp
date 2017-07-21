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

#include "bme280/bme280.hpp"

namespace sixtron {

#define SET__BITS__POS__0(reg__data, bitname, data) \
				((reg__data & ~(bitname##__MSK)) | \
				(data & bitname##__MSK))
#define GET__BITS(reg__data, bitname)  \
				((reg__data & (bitname##__MSK)) >> (bitname##__POS))
#define GET__BITS__POS__0(reg__data, bitname)  (reg__data & (bitname##__MSK))

/*!
 * @brief Default BME280 contructor
 *
 * @param i2c Instance of I2C
 * @param i2cAddress I2C address of the device
 *
 */
BME280::BME280(I2C* i2c, I2CAddress i2cAddress):
    _i2c(i2c), _i2cAddress(i2cAddress){
    _sensorMode = SensorMode::NORMAL;
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
   int ret;
   printf("Initializing the BME280...\n");
   if (!read_chip_id())
       return false;
   else {
       printf("Chip ID: 0x%X\n", 0x60);
       _chipId = 0x60;
   }

   ret = set_mode(SensorMode::NORMAL);
   if (ret > 0)
       return false;

   // wait for chip to wake up
   wait_ms(300);

   return true;
}


void BME280::update_settings(const uint8_t* reg_data){
    settings.osr_h = GET__BITS__POS__0(reg_data[0], CONTROL_HUMID);
    settings.osr_p = GET__BITS(reg_data[2], CONTROL_PRESS);
    settings.osr_t = GET__BITS(reg_data[2], CONTROL_TEMP);
    settings.filter = GET__BITS(reg_data[3], CONTROL_TEMP);
    settings.standby_time = GET__BITS(reg_data[3], STANDBY);
}

/*!
 *
 *
 */
int BME280::read_humidity(double* humidity){
    int bus_status = SUCCESS;


    return bus_status;
}

/*!
 *
 *
 */
int BME280::read_pressure(double* pressure){
    int bus_status = SUCCESS;


    return bus_status;
}

/*!
 * @brief Read sensor temperature
 *
 * @param temperature Pointer to the value of the temperature
 *
 * @return
 *        0 on succes,
 *        1 on failure
 *
 */
int BME280::read_temperature(double* temperature){
    int bus_status = SUCCESS;
    int32_t var1, var2;
    int32_t adc_T;
    bus_status = i2c_read_three_bytes(RegisterAddress::TEMP_MSB, &adc_T);
    if (bus_status != 0)
        return FAILURE;
    adc_T >>= 4;
    int16_t calib_dig_t[3];
    bus_status = i2c_read_vector(RegisterAddress::DIG_T1, calib_dig_t);
    if (bus_status != 0)
        return FAILURE;

    var1 = (((adc_T >> 3) - static_cast<int32_t>(calib_dig_t[0] << 1)) * 
           (static_cast<int32_t>(calib_dig_t[1]) >> 11));
    var2 = (((((adc_T >> FOUR_BITS_SHIFT) - (static_cast<int32_t>(calib_dig_t[0]))) *
           ((adc_T >> 4) - (static_cast<int32_t>(calib_dig_t[0])))) >> 12) *
           (static_cast<int32_t>(calib_dig_t[2]))) >> 14;
    if (temperature){
        *temperature = static_cast<double>(((var1 + var2) * 5 + 128) >> 8);
        *temperature /= 100.0;

        if (*temperature > TEMPERATURE_MAX)
            *temperature = TEMPERATURE_MAX;
        if (*temperature < TEMPERATURE_MIN)
            *temperature = TEMPERATURE_MIN;

        return SUCCESS;
    }
    return FAILURE;
}

int BME280::get_calib_data(bme280_calib_data_t* calib){
    int bus_status = SUCCESS;
    uint8_t calib_data[TEMP_PRESS_CALIB_DATA_LEN] = {0};
    bus_status = i2c_read_n_bytes(RegisterAddress::DIG_T1, calib_data, 
            TEMP_PRESS_CALIB_DATA_LEN);
    if (bus_status != 0)
        return FAILURE;
    
}

/*!
 *
 *
 */
int BME280::read_env_data(bme280_environment_t* env){
    int bus_status = SUCCESS;


    return bus_status;
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
int BME280::get_mode(SensorMode* mode){
    if (mode){
        *mode = _sensorMode;
        return SUCCESS;
    }
    return FAILURE;
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
int BME280::set_mode(SensorMode mode){
    int bus_status = SUCCESS;
    int8_t ctrl_meas = INIT_VALUE;
    bus_status = i2c_read_register(RegisterAddress::CONTROL_MEAS, &ctrl_meas);
    if (bus_status != 0)
        return FAILURE;
    ctrl_meas = (static_cast<int8_t>(ctrl_meas & CONTROL_MEAS__MSK)
            | static_cast<int8_t>(mode));
    bus_status = i2c_write_register(RegisterAddress::CONTROL_MEAS, ctrl_meas);
    if (bus_status == SUCCESS)
        _sensorMode = mode;
    else 
        printf("Error setting sensor mode\n");
    return bus_status;
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
   i2c_read_register(RegisterAddress::CHIP_ID, &chip_id);
   if (chip_id != 0x60){
       wait_ms(1000);
       i2c_read_register(RegisterAddress::CHIP_ID, &chip_id);
       return (chip_id != 0x60) ? false : true;
   }
   return true;
}

/*!
 * @brief Read register data
 *
 * @param registerAddress Address of the register
 * @param value Pointer to the value read from the register
 *
 * @return 
 *         0 on success,
 *         1 on failure
 */
int BME280::i2c_read_register(RegisterAddress registerAddress, int8_t* value){
	int bus_status = SUCCESS;
    static char data;
    data = static_cast<char>(registerAddress);
    bus_status = _i2c->write(static_cast<int>(_i2cAddress) << 1, &data, 1, true);
	if (bus_status != 0)
		return FAILURE;
	char* char_value = reinterpret_cast<char* >(value);
    bus_status = _i2c->read(static_cast<int>(_i2cAddress) << 1, char_value, 1);
	if (bus_status != 0)
        return FAILURE;
    return bus_status;
}

/*!
 * @brief Read successive registers data
 *
 * @note This function is useful to read memory-contiguous LSB/MSB registers
 *
 * @param registerAddress Address of the first register
 * @param value Pointer to the value read from the registers
 *
 * @return 
 *         0 on success,
 *         1 on failure
 */
int BME280::i2c_read_two_bytes(RegisterAddress registerAddress, int16_t* value){
    int bus_status = SUCCESS;
    static char data[2];
    data[0] = static_cast<char>(registerAddress);
    bus_status = _i2c->write(static_cast<int>(_i2cAddress) << 1, data, 1, true);
    if (bus_status != 0)
        return FAILURE;
    bus_status = _i2c->read(static_cast<int>(_i2cAddress) << 1, data, 2, false);
    if (bus_status != 0)
        return FAILURE;
    *value = static_cast<int16_t>((data[1] << EIGHT_BITS_SHIFT) | (0xFF & data[0]));
    return bus_status;
}

/*!
 * @brief Read three successive registers data
 *
 * @note This function is useful to read memory-contiguous LSB/MSB/XLSB registers
 *
 * @param registerAddress Address of the first register
 * @param value Pointer to the value read from the registers
 *
 * @return 
 *         0 on success,
 *         1 on failure
 */
int BME280::i2c_read_three_bytes(RegisterAddress registerAddress, int32_t* value){
    int bus_status = SUCCESS;
    static char data[3];
    data[0] = static_cast<char>(registerAddress);
    bus_status = _i2c->write(static_cast<int>(_i2cAddress) << 1, data, 1, true);
    if (bus_status != 0)
        return FAILURE;
    bus_status = _i2c->read(static_cast<int>(_i2cAddress) << 1, data, 3, false);
    if (bus_status != 0)
        return FAILURE;
    *value = static_cast<int32_t>((data[2] << SIXTEEN_BITS_SHIFT) | 
                                  (data[1] << EIGHT_BITS_SHIFT) |
                                  (data[0] & 0xFF));
    return SUCCESS;
}


/*!
 * @brief Read 16 bits signed vector (3 dimensions) continuous read
 *
 * @param registerAddress Address of the register containing the LSB 
 *                        of the first axis
 *        value Array to store the read values
 *
 * @return
 *         0 on success,
 *         1 on failure
 */
//TODO: use i2c_read_n_bytes instead?
int BME280::i2c_read_vector(RegisterAddress registerAddress, int16_t value[3]){
    int bus_status = SUCCESS;
    static char data[6];
    data[0] = static_cast<char>(registerAddress);
    bus_status = _i2c->write(static_cast<int>(_i2cAddress) << 1, data, 1, true);
    if (bus_status != 0)
        return FAILURE;
    bus_status = _i2c->read(static_cast<int>(_i2cAddress) << 1, data, 6, false);
    if (bus_status != 0)
        return FAILURE;
    for (int i = 0; i < 3; i++)
        value[i] = static_cast<int16_t>((data[2*i + 1] << EIGHT_BITS_SHIFT) | data[2*i]);
    return bus_status;
}

int BME280::i2c_read_n_bytes(RegisterAddress startAddress, int8_t* value, int n){
    int bus_status = SUCCESS;
    static char data;
    data = static_cast<char>(startAddress);
    bus_status = _i2c->write(static_cast<_i2cAddress) << 1, data, 1, true);
    if (bus_status != 0)
        return FAILURE;
    bus_status = _i2c->read(static_cast<_i2cAddress) << 1, value, n, false);
    if (bus_status != 0)
        return FAILURE;
    for (int i = 0; i < n; i++)
        value[i] = static_cast<int16_t>((data[2*i + 1] << EIGHT_BITS_SHIFT) | data[2*i]);
    return bus_status;
}

/*!
 * @brief Write to a register
 *
 * @param registerAddress Address of the register to write to
 * @param value Data to store in the register
 *
 * @return 
 *         0 on success,
 *         1 on failure
 */
int BME280::i2c_write_register(RegisterAddress registerAddress, int8_t value){
	int bus_status = SUCCESS;
    static char data[2];
    data[0] = static_cast<char>(registerAddress);
    data[1] = static_cast<char>(value);
    bus_status = _i2c->write(static_cast<int>(_i2cAddress) << 1, data, 2);
	if (bus_status != 0)
        return FAILURE;
    return bus_status;
}

}
