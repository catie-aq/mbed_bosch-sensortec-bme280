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
#ifndef BME280_H_
#define BME280_H_

namespace sixtron {

typedef struct {
    uint16_t DIG_T1;
    int16_t  DIG_T2;
    int16_t  DIG_T3;

    uint16_t DIG_P1;
    int16_t  DIG_P2;
    int16_t  DIG_P3;
    int16_t  DIG_P4;
    int16_t  DIG_P5;
    int16_t  DIG_P6;
    int16_t  DIG_P7;
    int16_t  DIG_P8;
    int16_t  DIG_P9;

    uint8_t  DIG_H1;
    int16_t  DIG_H2;
    uint8_t  DIG_H3;
    int16_t  DIG_H4;
    int16_t  DIG_H5;
    int8_t   DIG_H6;
} bme280_calib_t;

typedef struct {
    float humidity;
    float pressure;
    float temperature;
} bme280_environment_t;

class bme280
{
public:
    /* I2C addresses */
    enum class I2CAddress : char {
        Adress = 0x77
    };

    enum class RegisterAddress : char {
        CHIP_ID   = 0xD0
        VERSION  = 0xD1
        RESET    = 0xE0
        /* Calibration register */
        DIG_T1   = 0x88
        DIG_T2   = 0x8A
        DIG_T3   = 0x8D

        DIG_P1              = 0x8E,
        DIG_P2              = 0x90,
        DIG_P3              = 0x92,
        DIG_P4              = 0x94,
        DIG_P5              = 0x96,
        DIG_P6              = 0x98,
        DIG_P7              = 0x9A,
        DIG_P8              = 0x9C,
        DIG_P9              = 0x9E,

        DIG_H1              = 0xA1,
        DIG_H2              = 0xE1,
        DIG_H3              = 0xE3,
        DIG_H4              = 0xE4,
        DIG_H5              = 0xE5,
        DIG_H6              = 0xE7,

        CONTROLHUMID       = 0xF2,
        STATUS             = 0XF3,
        CONTROL            = 0xF4,
        CONFIG             = 0xF5,
        PRESSURE           = 0xF7,
        TEMP               = 0xFA,
        HUMID              = 0xFD
    };

    enum class SensorMode : char {
        SLEEP  = 0b00,
        FORCED = 0b01,
        NORMAL = 0b11
    };

    enum class SensorSampling : char {
        NONE  = 0b000,
        X1    = 0b001,
        X2    = 0b010,
        X4    = 0b011,
        X8    = 0b100,
        X16   = 0b101
    };

    enum class SensorFilter : char {
        OFF = 0b000,
        X2  = 0b001,
        X4  = 0b010,
        X8  = 0b011,
        X16 = 0b100
    };

    enum class StandbyDuration : char {
        MS_0_5   = 0b000,
        MS_62_5  = 0b001,
        MS_125   = 0b010,
        MS_250   = 0b011,
        MS_500   = 0b100,
        MS_1000  = 0b101,
        MS_10    = 0b110,
        MS_20    = 0b111,
    };

    BME280(I2C* i2c, I2CAddress address = I2CAddress::Address);

    int power_off();
    int resume();

    bool initialize();

    int read_humidity(float* humidity);
    int read_pressure(float* pressure);
    int read_temperature(float* temperature);
    int read_env_data(bme280_environment_t* env);

    char chip_id() { return _chipId; }

private:
    int i2c_read_register(RegisterAddress registerAddress, int8_t* value);
    int i2c_read_two_bytes(RegisterAddress registerAddress, int16_t* value);
    int i2c_read_vector(RegisterAddress registerAddress, int16_t value[3]);
    int i2c_write_register(RegisterAddress registerAddress, int8_t value);

    char _chipId = 0;
};

} // namespace sixtron

#endif // BME280_H_