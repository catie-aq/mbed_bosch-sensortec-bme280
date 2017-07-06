/*
 * Copyright (c) 2016, CATIE, All Rights Reserved
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
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calib_data;


class bme280
{
public:
    /* I2C addresses */
    enum class I2CAddress : char {
        Adress = 0x77
    }

    enum class RegisterAddress : char {
        ChipID   = 0xD0
        version  = 0xD1
        reset    = 0xE0
        /* Calibration register */
        dig_T1   = 0x88
        dig_T2   = 0x8A
        dig_T3   = 0x8D

        dig_P1              = 0x8E,
        dig_P2              = 0x90,
        dig_P3              = 0x92,
        dig_P4              = 0x94,
        dig_P5              = 0x96,
        dig_P6              = 0x98,
        dig_P7              = 0x9A,
        dig_P8              = 0x9C,
        dig_P9              = 0x9E,

        dig_H1              = 0xA1,
        dig_H2              = 0xE1,
        dig_H3              = 0xE3,
        dig_H4              = 0xE4,
        dig_H5              = 0xE5,
        dig_H6              = 0xE7,

        controlhumid       = 0xF2,
        status             = 0XF3,
        control            = 0xF4,
        config             = 0xF5,
        pressure           = 0xF7,
        temp               = 0xFA,
        humid              = 0xFD
    }

    enum class sensor_mode : char {
        mode_sleep  = 0b00,
        mode_forced = 0b01,
        mode_normal = 0b11
    }

    enum class sensor_sampling : char {
        sampling_none  = 0b000,
        sampling_X1    = 0b001,
        sampling_X2    = 0b010,
        sampling_X4    = 0b011,
        sampling_X8    = 0b100,
        sampling_X16   = 0b101
    }

    enum class sensor_filter : char {
        filter_off = 0b000,
        filter_X2  = 0b001,
        filter_X4  = 0b010,
        filter_X8  = 0b011,
        filter_X16 = 0b100
    }

    enum class stanby_duration : char {
        stanby_ms_0_5   = 0b000,
        stanby_ms_62_5  = 0b001,
        stanby_ms_125   = 0b010,
        stanby_ms_250   = 0b011,
        stanby_ms_500   = 0b100,
        stanby_ms_1000  = 0b101,
        stanby_ms_10    = 0b110,
        stanby_ms_20    = 0b111,
    }

    BME280(I2C * i2c, I2CAddress address = I2CAddress::Address, int hz = 400000);

    bool initialize(sensor_mode mode = mode_normal)

    void readtemp(float* temp);
    void readhumid(float* humid);
    void readpressure(float* pressure);

private:

   char _chipId = 0;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_LIBTEMPLATE_H_
