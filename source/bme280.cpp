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

#include "bme280.hpp"

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
    *value = (int16_t) (data[1] << EIGHT_BITS_SHIFT) | (0xFF & data[0]);
    return bus_status;
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
        value[i] = (data[2*i + 1] << EIGHT_BITS_SHIFT) | (0xFF & data[2*i]);
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