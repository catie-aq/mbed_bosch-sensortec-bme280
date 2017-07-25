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
#ifndef CATIE_SIXTRON_BME280_H_
#define CATIE_SIXTRON_BME280_H_

#include "mbed.h"

namespace sixtron {

/**\name MACROS DEFINITIONS                      */
#define INIT_VALUE                         0
#define SUCCESS                            0
#define FAILURE                            1
/** Bit value manipulation                       */
#define ZERO                               0
#define ONE                                1
#define ONE_BIT_SHIFT                      1        
#define TWO_BITS_SHIFT                     2        
#define FOUR_BITS_SHIFT                    4
#define EIGHT_BITS_SHIFT                   8        
#define SIXTEEN_BITS_SHIFT                 16

#define CONTROL_MEAS__MSK                  0xFC

#define CONTROL_HUMID__MSK                 0x07
#define CONTROL_HUMID__POS                 0
#define CONTROL_PRESS__MSK                 0x1C
#define CONTROL_PRESS__POS                 2
#define CONTROL_TEMP__MSK                  0x1C
#define CONTROL_TEMP__POS                  5
#define FILTER__MSK                        0x1C
#define FILTER__POS                        2
#define STANDBY__MSK                       0xE0
#define STANDBY__POS                       5

#define TEMPERATURE_MIN                    -40
#define TEMPERATURE_MAX                    85

#define TEMP_PRESS_CALIB_DATA_LEN          26

/*!
 * @brief BME280 environment data structure: temperature, humidity, pressure
 * @note Humidity measured in 
 * @note Pressure measured in hPa
 * @note Temperature measured in °C
 */
typedef struct {
    double humidity;
    double pressure;
    double temperature;
} bme280_environment_t;

/*!
 * @brief BME280 uncompensated environment data structure:
 * temperature, humidity, pressure
 */
typedef struct {
    uint32_t humidity;
    uint32_t pressure;
    uint32_t temperature;
} bme280_uncomp_data;

/*!
 * @brief BME280 sensor settings structure: oversampling and filter settings
 */
typedef struct {
    uint8_t osr_h;
    uint8_t osr_p;
    uint8_t osr_t;
    uint8_t filter;
    uint8_t standby_time;
} bme280_settings_t;

class BME280
{
public:
    /* I2C addresses */
    enum class I2CAddress : char {
        Address1           = 0x76,
        Address2           = 0x77
    };

    enum class RegisterAddress : char {
        CHIP_ID            = 0xD0,
        VERSION            = 0xD1,
        RESET              = 0xE0,

        DIG_T1             = 0x88,
        DIG_T2             = 0x8A,
        DIG_T3             = 0x8D,
        DIG_P1             = 0x8E,
        DIG_P2             = 0x90,
        DIG_P3             = 0x92,
        DIG_P4             = 0x94,
        DIG_P5             = 0x96,
        DIG_P6             = 0x98,
        DIG_P7             = 0x9A,
        DIG_P8             = 0x9C,
        DIG_P9             = 0x9E,
        DIG_H1             = 0xA1,
        DIG_H2             = 0xE1,
        DIG_H3             = 0xE3,
        DIG_H4             = 0xE4,
        DIG_H5             = 0xE5,
        DIG_H6             = 0xE7,

        CONTROL_HUMID      = 0xF2,
        STATUS             = 0XF3,
        CONTROL_MEAS       = 0xF4,
        CONFIG             = 0xF5,
        PRESSURE           = 0xF7,
        TEMP_MSB           = 0xFA,
        TEMP_LSB           = 0xFB,
        TEMP_XLSB          = 0xFC,
        HUMID_MSB          = 0xFD,
        HUMID_LSB          = 0xFE
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

    BME280(I2C* i2c, I2CAddress address = I2CAddress::Address1);
    bool initialize();

    int power_off();
    int resume();

    void update_settings(const uint8_t* reg_data);

    int read_humidity(double* humidity);
    int read_pressure(double* pressure);
    int read_temperature(double* temperature);
    int read_env_data(bme280_environment_t* env);

    int set_mode(SensorMode mode);
    int get_mode(SensorMode* mode);

    char chip_id() { return _chipId; }

private:
    char _chipId = 0;
    I2C* _i2c;
    I2CAddress _i2cAddress;
    SensorMode _sensorMode;
    bme280_settings_t settings = {0, 0, 0, 0, 0};

    bool read_chip_id();
    int i2c_read_register(RegisterAddress registerAddress, int8_t* value);
    int i2c_read_two_bytes(RegisterAddress registerAddress, int16_t* value);
    int i2c_read_three_bytes(RegisterAddress registerAddress, int32_t* value);
    int i2c_read_vector(RegisterAddress registerAddress, int16_t value[3]);
    int i2c_read_n_bytes(RegisterAddress startAddress, int8_t* value, int n);
    int i2c_write_register(RegisterAddress registerAddress, int8_t value);
};

} // namespace sixtron
#endif // CATIE_SIXTRON_BME280_H_