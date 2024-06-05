/**
 * Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    bme68xLibrary.h
 * @date    11 Jan 2023
 * @version 1.2.40408
 *
 */

#ifndef BME68X_LIBRARY_H
#define BME68X_LIBRARY_H

#include <string.h>
#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"

#include "bme68x/bme68x.h"

#define BME68X_ERROR            INT8_C(-1)
#define BME68X_WARNING          INT8_C(1)

/**
 * Datatype working as an interface descriptor
 */
typedef union
{
    struct
    {
        TwoWire *wireobj;
        uint8_t i2cAddr;
    } i2c;
    struct
    {
        SPIClass *spiobj;
        uint8_t cs;
    } spi;
} bme68xScommT;

/** Datatype to keep consistent with camel casing */
typedef struct bme68x_data          bme68xData;
typedef struct bme68x_dev           bme68xDev;
typedef enum   bme68x_intf          bme68xIntf;
typedef struct bme68x_conf          bme68xConf;
typedef struct bme68x_heatr_conf    bme68xHeatrConf;

/**
 * @brief Function that implements the default microsecond delay callback
 * @param periodUs : Duration of the delay in microseconds
 * @param intfPtr  : Pointer to the interface descriptor
 */
void bme68xDelayUs(uint32_t periodUs, void *intfPtr);

/**
 * @brief Function that implements the default SPI write transaction
 * @param regAddr : Register address of the sensor
 * @param regData : Pointer to the data to be written to the sensor
 * @param length   : Length of the transfer
 * @param intfPtr : Pointer to the interface descriptor
 * @return 0 if successful, non-zero otherwise
 */
int8_t bme68xSpiWrite(uint8_t regAddr, const uint8_t *regData, uint32_t length, void *intfPtr);

/**
 * @brief Function that implements the default SPI read transaction
 * @param regAddr : Register address of the sensor
 * @param regData : Pointer to the data to be read from the sensor
 * @param length   : Length of the transfer
 * @param intfPtr : Pointer to the interface descriptor
 * @return 0 if successful, non-zero otherwise
 */
int8_t bme68xSpiRead(uint8_t regAddr, uint8_t *regData, uint32_t length, void *intfPtr);

/**
 * @brief Function that implements the default I2C write transaction
 * @param regAddr : Register address of the sensor
 * @param regData : Pointer to the data to be written to the sensor
 * @param length   : Length of the transfer
 * @param intfPtr : Pointer to the interface descriptor
 * @return 0 if successful, non-zero otherwise
 */
int8_t bme68xI2cWrite(uint8_t regAddr, const uint8_t *regData, uint32_t length, void *intfPtr);

/**
 * @brief Function that implements the default I2C read transaction
 * @param regAddr : Register address of the sensor
 * @param regData : Pointer to the data to be written to the sensor
 * @param length   : Length of the transfer
 * @param intfPtr : Pointer to the interface descriptor
 * @return 0 if successful, non-zero otherwise
 */
int8_t bme68xI2cRead(uint8_t regAddr, uint8_t *regData, uint32_t length, void *intfPtr);

class Bme68x
{
public:
    /** Stores the BME68x sensor APIs error code after an execution */
    int8_t status;

    /**
     * Class constructor
     */
    Bme68x(void);

    /**
     * @brief Function to initialize the sensor based on custom callbacks
     * @param intf     : BME68X_SPI_INTF or BME68X_I2C_INTF interface
     * @param read     : Read callback
     * @param write    : Write callback
     * @param idleTask : Delay or Idle function
     * @param intfPtr : Pointer to the interface descriptor
     */
    void begin(bme68xIntf intf, bme68x_read_fptr_t read, bme68x_write_fptr_t write,
            bme68x_delay_us_fptr_t idleTask, void *intfPtr);

    /**
     * @brief Function to initialize the sensor based on the Wire library
     * @param i2cAddr  : The I2C address the sensor is at
     * @param i2c      : The TwoWire object
     * @param idleTask : Delay or Idle function
     */
    void begin(uint8_t i2cAddr, TwoWire &i2c, bme68x_delay_us_fptr_t idleTask = bme68xDelayUs);

    /**
     * @brief Function to initialize the sensor based on the SPI library
     * @param chipSelect : The chip select pin for SPI communication
     * @param spi        : The SPIClass object
     * @param idleTask   : Delay or Idle function
     */
    void begin(uint8_t chipSelect, SPIClass &spi, bme68x_delay_us_fptr_t idleTask = bme68xDelayUs);

    /**
     * @brief Function to read a register
     * @param regAddr : Register address
     * @return Data at that register
     */
    uint8_t readReg(uint8_t regAddr);

    /**
     * @brief Function to read multiple registers
     * @param regAddr : Start register address
     * @param regData : Pointer to store the data
     * @param length  : Number of registers to read
     */
    void readReg(uint8_t regAddr, uint8_t *regData, uint32_t length);

    /**
     * @brief Function to write data to a register
     * @param regAddr : Register addresses
     * @param regData : Data for that register
     */
    void writeReg(uint8_t regAddr, uint8_t regData);

    /**
     * @brief Function to write multiple registers
     * @param regAddr : Pointer to the register addresses
     * @param regData : Pointer to the data for those registers
     * @param length  : Number of register to write
     */
    void writeReg(uint8_t *regAddr, const uint8_t *regData, uint32_t length);

    /**
     * @brief Function to trigger a soft reset
     */
    void softReset(void);

    /**
     * @brief Function to set the ambient temperature for better configuration
     * @param temp : Temperature in degree Celsius. Default is 25 deg C
     */
    void setAmbientTemp(int8_t temp = 25);

    /**
     * @brief Function to get the measurement duration in microseconds
     * @param opMode : Operation mode of the sensor. Attempts to use the last one if nothing is set
     * @return Temperature, Pressure, Humidity measurement time in microseconds
     */
    uint32_t getMeasDur(uint8_t opMode = BME68X_SLEEP_MODE);

    /**
     * @brief Function to set the operation mode
     * @param opMode : BME68X_SLEEP_MODE, BME68X_FORCED_MODE, BME68X_PARALLEL_MODE, BME68X_SEQUENTIAL_MODE
     */
    void setOpMode(uint8_t opMode);

    /**
     * @brief Function to get the operation mode
     * @return Operation mode : BME68X_SLEEP_MODE, BME68X_FORCED_MODE, BME68X_PARALLEL_MODE, BME68X_SEQUENTIAL_MODE
     */
    uint8_t getOpMode(void);

    /**
     * @brief Function to get the Temperature, Pressure and Humidity over-sampling
     * @param osHum  : BME68X_OS_NONE to BME68X_OS_16X
     * @param osTemp : BME68X_OS_NONE to BME68X_OS_16X
     * @param osPres : BME68X_OS_NONE to BME68X_OS_16X
     */
    void getTPH(uint8_t &osHum, uint8_t &osTemp, uint8_t &osPres);

    /**
     * @brief Function to set the Temperature, Pressure and Humidity over-sampling.
     *        Passing no arguments sets the defaults.
     * @param osTemp : BME68X_OS_NONE to BME68X_OS_16X
     * @param osPres : BME68X_OS_NONE to BME68X_OS_16X
     * @param osHum  : BME68X_OS_NONE to BME68X_OS_16X
     */
    void setTPH(uint8_t osTemp = BME68X_OS_2X, uint8_t osPres = BME68X_OS_16X, uint8_t osHum = BME68X_OS_1X);

    /**
     * @brief Function to get the filter configuration
     * @return BME68X_FILTER_OFF to BME68X_FILTER_SIZE_127
     */
    uint8_t getFilter(void);

    /**
     * @brief Function to set the filter configuration
     * @param filter : BME68X_FILTER_OFF to BME68X_FILTER_SIZE_127
     */
    void setFilter(uint8_t filter = BME68X_FILTER_OFF);

    /**
     * @brief Function to get the sleep duration during Sequential mode
     * @return BME68X_ODR_NONE to BME68X_ODR_1000_MS
     */
    uint8_t getSeqSleep(void);

    /**
     * @brief Function to set the sleep duration during Sequential mode
     * @param odr : BME68X_ODR_NONE to BME68X_ODR_1000_MS
     */
    void setSeqSleep(uint8_t odr = BME68X_ODR_0_59_MS);

    /**
     * @brief Function to set the heater profile for Forced mode
     * @param temp : Heater temperature in degree Celsius
     * @param dur  : Heating duration in milliseconds
     */
    void setHeaterProf(uint16_t temp, uint16_t dur);

    /**
     * @brief Function to set the heater profile for Sequential mode
     * @param temp       : Heater temperature profile in degree Celsius
     * @param dur        : Heating duration profile in milliseconds
     * @param profileLen : Length of the profile
     */
    void setHeaterProf(uint16_t *temp, uint16_t *dur, uint8_t profileLen);

    /**
     * @brief Function to set the heater profile for Parallel mode
     * @param temp           : Heater temperature profile in degree Celsius
     * @param mul            : Profile of number of repetitions
     * @param sharedHeatrDur : Shared heating duration in milliseconds
     * @param profileLen     : Length of the profile
     */
    void setHeaterProf(uint16_t *temp, uint16_t *mul, uint16_t sharedHeatrDur, uint8_t profileLen);

    /**
     * @brief Function to fetch data from the sensor into the local buffer
     * @return Number of new data fields
     */
    uint8_t fetchData(void);

    /**
     * @brief Function to get a single data field
     * @param data : Structure where the data is to be stored
     * @return Number of new fields remaining
     */
    uint8_t getData(bme68xData &data);
	
	 /**
     * @brief Function to get whole sensor data
     * @return Sensor data
     */
    bme68xData* getAllData(void);

    /**
	 * @brief Function to get the BME68x heater configuration
	 */
    const bme68xHeatrConf& getHeaterConfiguration(void);

    /**
     * @brief Function to retrieve the sensor's unique ID
     * @return Unique ID
     */
    uint32_t getUniqueId(void);

    /**
     * @brief Function to get the error code of the interface functions
     * @return Interface return code
     */
    BME68X_INTF_RET_TYPE intfError(void);

    /**
     * @brief Function to check if an error / warning has occurred
     * @return -1 if an error occurred, 1 if warning occured else 0
     */
    int8_t checkStatus(void);

    /**
     * @brief Function to get a brief text description of the error
     * @return Returns a string describing the error code
     */
    String statusString(void);

private:
    /** Datatype to keep consistent with camel casing
     * Datastructure to hold sensor settings
     */
    bme68xScommT comm;
    bme68xDev bme6;
    bme68xConf conf;
    bme68xHeatrConf heatrConf;
    bme68xData sensorData[3];
    uint8_t nFields, iFields;
    uint8_t lastOpMode;
};

#endif /* BME68X_CLASS_H */
