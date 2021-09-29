/**
 * Copyright (C) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 */

#include "Arduino.h"
#include "bme68xLibrary.h"

#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)
#define MEAS_DUR 140

#ifndef PIN_CS
#define PIN_CS SS
#endif

Bme68x bme;

/**
 * @brief Initializes the sensor and hardware settings
 */
void setup(void)
{
	SPI.begin();
	Serial.begin(115200);
	
	while (!Serial)
		delay(10);

	/* initializes the sensor based on SPI library */
	bme.begin(PIN_CS, SPI);

	if(bme.checkStatus())
	{
		if (bme.checkStatus() == BME68X_ERROR)
		{
			Serial.println("Sensor error:" + bme.statusString());
			return;
		}
		else if (bme.checkStatus() == BME68X_WARNING)
		{
			Serial.println("Sensor Warning:" + bme.statusString());
		}
	}
	
	/* Set the default configuration for temperature, pressure and humidity */
	bme.setTPH();

	/* Heater temperature in degree Celsius */
	uint16_t tempProf[10] = { 320, 100, 100, 100, 200, 200, 200, 320, 320,
			320 };
	/* Multiplier to the shared heater duration */
	uint16_t mulProf[10] = { 5, 2, 10, 30, 5, 5, 5, 5, 5, 5 };
	/* Shared heating duration in milliseconds */
	uint16_t sharedHeatrDur = MEAS_DUR - (bme.getMeasDur(BME68X_PARALLEL_MODE) / 1000);

	bme.setHeaterProf(tempProf, mulProf, sharedHeatrDur, 10);
	bme.setOpMode(BME68X_PARALLEL_MODE);

	Serial.println("TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%), Gas resistance(ohm), Status, Gas index");
}

void loop(void)
{
	bme68xData data;
	uint8_t nFieldsLeft = 0;

	/* data being fetched for every 140ms */
	delay(MEAS_DUR);

	if (bme.fetchData())
	{
		do
		{
			nFieldsLeft = bme.getData(data);
			if (data.status == NEW_GAS_MEAS)
			{
				Serial.print(String(millis()) + ", ");
				Serial.print(String(data.temperature) + ", ");
				Serial.print(String(data.pressure) + ", ");
				Serial.print(String(data.humidity) + ", ");
				Serial.print(String(data.gas_resistance) + ", ");
				Serial.print(String(data.status, HEX) + ", ");
				Serial.println(data.gas_index);
			}
		} while (nFieldsLeft);
	}
}
