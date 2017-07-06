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

BME280::BME280(I2C * i2c, I2CAddress address):
        _i2cAddress(address){
    _i2c = i2c;
{

bool BME280:initialize(sensor_mode mode)
{
    char reg = 0;
    printf("Initializing BME280 ... \n");
    i2c_read_register(RegisterAddress::chipID, &reg);
    if (reg != 0x60) {
        return false;
    }

    _chipId = reg
}

}

} // namespace sixtron
