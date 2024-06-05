/**
 *
 * Copyright (C) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/**
 * bme688_dev_kit.ino :
 * This is an example to log data using the BME688 development 
 * kit which has been designed to work with Adafruit ESP32 Feather Board
 * For more information visit :
 * https://www.bosch-sensortec.com/software-tools/software/bme688-software/
 */

#include "Arduino.h"
#include "bme68xLibrary.h"
#include "commMux.h"

#include <SdFat.h>
#include <Esp.h>

/* Macros used in BME68x_datalogger module */
#define N_KIT_SENS 8
#define SD_PIN_CS 33
#define PANIC_LED LED_BUILTIN
#define PANIC_DUR 1000
/* measurement duration */
#define MEAS_DUR 140
#define LOG_FILE_NAME "/BME688_Datalogger_Log.csv"

/* Declaration of variables */
Bme68x bme[N_KIT_SENS];
commMux commSetup[N_KIT_SENS];
uint8_t lastMeasindex[N_KIT_SENS] = {0};
bme68xData sensorData[N_KIT_SENS] = {0};
String logHeader;
uint32_t lastLogged = 0;

static SdFat sd;

/**
 * @brief Initializes the sensor and hardware settings
 * Initializes the SD card module
 */
void setup(void) {
  Serial.begin(115200);
  /* Initiate SPI communication  */
  commMuxBegin(Wire, SPI);
  pinMode(PANIC_LED, OUTPUT);
  delay(100);

  /* Setting SD Card */
  if (!sd.begin(SD_PIN_CS, SPI_EIGHTH_SPEED)) {
    Serial.println("SD Card not found");
    panicLeds();
  } else {
    sd.remove(LOG_FILE_NAME);
    File file;
    
    if (!file.open(LOG_FILE_NAME, (O_RDWR | O_CREAT))) {
      Serial.println("Failed to open file for writing");
      panicLeds();
    }
    /* Parameters for logging in the file */
    logHeader = "TimeStamp(ms),Sensor Index,Temperature(deg "
                "C),Pressure(Pa),Humidity(%),Gas Resistance(ohm),Gas "
                "Index,Meas Index,idac,Status,Gas Valid,Heater Stable";

    if (file.println(logHeader)) {
      Serial.println(logHeader);
      file.close();
    } else {
      panicLeds();
    }
    logHeader = "";
  }

  /* Communication interface set for all the 8 sensors in the development kit */
  for (uint8_t i = 0; i < N_KIT_SENS; i++) {
    commSetup[i] = commMuxSetConfig(Wire, SPI, i, commSetup[i]);
    bme[i].begin(BME68X_SPI_INTF, commMuxRead, commMuxWrite, commMuxDelay,
                 &commSetup[i]);
    if(bme[i].checkStatus()) {
      Serial.println("Initializing sensor " + String(i) + " failed with error " + bme[i].statusString());
      panicLeds();
    }
  }

  /* Setting the default heater profile configuration */
  for (uint8_t i = 0; i < N_KIT_SENS; i++) {
    bme[i].setTPH();

    /* Heater temperature in degree Celsius as per the suggested heater profile
     */
    uint16_t tempProf[10] = {320, 100, 100, 100, 200, 200, 200, 320, 320, 320};
    /* Multiplier to the shared heater duration */
    uint16_t mulProf[10] = {5, 2, 10, 30, 5, 5, 5, 5, 5, 5};
    /* Shared heating duration in milliseconds */
    uint16_t sharedHeatrDur =
        MEAS_DUR - (bme[i].getMeasDur(BME68X_PARALLEL_MODE) / INT64_C(1000));

    bme[i].setHeaterProf(tempProf, mulProf, sharedHeatrDur, 10);

    /* Parallel mode of sensor operation */
    bme[i].setOpMode(BME68X_PARALLEL_MODE);
  }
}

void loop(void) {
  uint8_t nFieldsLeft = 0;
  int16_t indexDiff;
  bool newLogdata = false;
  /* Control loop for data acquisition - checks if the data is available */
  if ((millis() - lastLogged) >= MEAS_DUR) {

    lastLogged = millis();
    for (uint8_t i = 0; i < N_KIT_SENS; i++) {
      if (bme[i].fetchData()) {
        do {
          nFieldsLeft = bme[i].getData(sensorData[i]);
          /* Check if new data is received */
          if (sensorData[i].status & BME68X_NEW_DATA_MSK) {
            /* Inspect miss of data index */
            indexDiff =
                (int16_t)sensorData[i].meas_index - (int16_t)lastMeasindex[i];
            if (indexDiff > 1) {

              Serial.println("Skip I:" + String(i) +
                             ", DIFF:" + String(indexDiff) +
                             ", MI:" + String(sensorData[i].meas_index) +
                             ", LMI:" + String(lastMeasindex[i]) +
                             ", S:" + String(sensorData[i].status, HEX));
              panicLeds();
            }
            lastMeasindex[i] = sensorData[i].meas_index;

            logHeader += millis();
            logHeader += ",";
            logHeader += i;
            logHeader += ",";
            logHeader += sensorData[i].temperature;
            logHeader += ",";
            logHeader += sensorData[i].pressure;
            logHeader += ",";
            logHeader += sensorData[i].humidity;
            logHeader += ",";
            logHeader += sensorData[i].gas_resistance;
            logHeader += ",";
            logHeader += sensorData[i].gas_index;
            logHeader += ",";
            logHeader += sensorData[i].meas_index;
            logHeader += ",";
            logHeader += sensorData[i].idac;
            logHeader += ",";
            logHeader += String(sensorData[i].status, HEX);
            logHeader += ",";
            logHeader += sensorData[i].status & BME68X_GASM_VALID_MSK;
            logHeader += ",";
            logHeader += sensorData[i].status & BME68X_HEAT_STAB_MSK;
            logHeader += "\r\n";
            newLogdata = true;
          }
        } while (nFieldsLeft);
      }
    }
  }

  if (newLogdata) {
    newLogdata = false;

    digitalWrite(PANIC_LED, HIGH);
    
    appendFile(logHeader);
    logHeader = "";

    digitalWrite(PANIC_LED, LOW);
  }
}

/*!
 * @brief Configuring the sensor with digital pin 13 as
 * an output and toggles it at one second pace
 */
static void panicLeds(void) {
  while (1) {
    digitalWrite(PANIC_LED, HIGH);
    delay(PANIC_DUR);
    digitalWrite(PANIC_LED, LOW);
    delay(PANIC_DUR);
  }
}

/*!
 * @brief Writing the sensor data to the log file(csv)
 * @param sensorData
 */
static void writeFile(String sensorData) {

  File file;

  if (!file.open(LOG_FILE_NAME, (O_RDWR | O_AT_END))) {
    Serial.println("Failed to open file for writing");
    panicLeds();
  }
  if (file.print(sensorData)) {
    Serial.print(sensorData);
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

/*!
 * @brief Appending the sensor data into the log file(csv)
 * @param sensorData
 */
static void appendFile(String sensorData) {
  File file;

  if (!file.open(LOG_FILE_NAME, (O_RDWR | O_AT_END))) {
    Serial.println("Failed to open file for appending");
    panicLeds();
  }
  if (file.print(sensorData)) {
    Serial.print(sensorData);
  } else {
    Serial.println("Write append");
  }
  file.close();
}
