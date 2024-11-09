/**
 * Copyright (C) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 */

#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bme68xLibrary.h"
#include "I2Cbus.hpp"

#define BME688_ADD 0x77

static const char* TAG = "main";

Bme68x bme;

int8_t read_bytes_wrapper(uint8_t a_register, uint8_t *data, uint32_t len, void *intfPtr) {
  return static_cast<I2C_t *>(intfPtr)->readBytes(BME688_ADD, a_register, len, data)==ESP_OK  ? 0 : -1;
}

int8_t write_bytes_wrapper(uint8_t a_register, const uint8_t *data, uint32_t len,
                                                    void *intfPtr) {
  return static_cast<I2C_t *>(intfPtr)->writeBytes(BME688_ADD, a_register, len, data)==ESP_OK ? 0 : -1;
}

uint32_t IRAM_ATTR millis() { return (uint32_t) (esp_timer_get_time() / 1000ULL); }
void IRAM_ATTR delay(uint32_t ms) { vTaskDelay(ms / portTICK_PERIOD_MS); }
uint32_t IRAM_ATTR micros() { return (uint32_t) esp_timer_get_time(); }

void delay_microseconds_safe(uint32_t us) {  // avoids CPU locks that could trigger WDT or affect WiFi/BT stability
  uint32_t start = micros();

  const uint32_t lag = 5000;  // microseconds, specifies the maximum time for a CPU busy-loop.
                              // it must be larger than the worst-case duration of a delay(1) call (hardware tasks)
                              // 5ms is conservative, it could be reduced when exact BT/WiFi stack delays are known
  if (us > lag) {
    delay((us - lag) / 1000UL);  // note: in disabled-interrupt contexts delay() won't actually sleep
    while (micros() - start < us - lag)
      delay(1);  // in those cases, this loop allows to yield for BT/WiFi stack tasks
  }
  while (micros() - start < us)  // fine delay the remaining usecs
    ;
}

void delay_us(uint32_t period, void *intfPtr) {
  delay_microseconds_safe(period);
}

/**
 * @brief Initializes the sensor and hardware settings
 */
void setup(void)
{
	i2c0.begin(GPIO_NUM_3,GPIO_NUM_0);
	
  /* initializes the sensor based on I2C library */
	bme.begin(BME68X_I2C_INTF, read_bytes_wrapper, write_bytes_wrapper, delay_us, (void *) &i2c0);

	if(bme.checkStatus())
	{
		if (bme.checkStatus() == BME68X_ERROR)
		{
			ESP_LOGI(TAG, "Sensor error: %d", bme.status);
			return;
		}
		else if (bme.checkStatus() == BME68X_WARNING)
		{
			ESP_LOGI(TAG, "Sensor Warning: %d", bme.status);
		}
	}
	
	/* Set the default configuration for temperature, pressure and humidity */
	bme.setTPH();

	/* Set the heater configuration to 300 deg C for 100ms for Forced mode */
	bme.setHeaterProf(300, 100);

	ESP_LOGI(TAG, "TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status");
}

void loop(void)
{
	bme68xData data;

	bme.setOpMode(BME68X_FORCED_MODE);
	delay_microseconds_safe(bme.getMeasDur());

	if (bme.fetchData())
	{
		bme.getData(data);
		ESP_LOGI(TAG, "%lu, %f, %f, %f, %f, %d", millis(), data.temperature, data.pressure, data.humidity, data.gas_resistance, data.status);
	}
}

void loop_task(void *pv_params) {
  setup();
  while (true) {
    loop();
  }
}

extern "C" void app_main()
{
	ESP_LOGI(TAG, "starting");
	xTaskCreate(loop_task, "loopTask", 8192, nullptr, 1, NULL);
}