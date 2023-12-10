/*
 * temperature_sensor.ino
 *
 * This example shows how to:
 * 1. define a temperature sensor accessory and its characteristics (in my_accessory.c).
 * 2. report the sensor value to HomeKit (just random value here, you need to change it to your real sensor value).
 *
 *  Created on: 2020-05-15
 *      Author: Mixiaoxiao (Wang Bin)
 *
 * Note:
 *
 * You are recommended to read the Apple's HAP doc before using this library.
 * https://developer.apple.com/support/homekit-accessory-protocol/
 *
 * This HomeKit library is mostly written in C,
 * you can define your accessory/service/characteristic in a .c file,
 * since the library provides convenient Macro (C only, CPP can not compile) to do this.
 * But it is possible to do this in .cpp or .ino (just not so conveniently), do it yourself if you like.
 * Check out homekit/characteristics.h and use the Macro provided to define your accessory.
 *
 * Generally, the Arduino libraries (e.g. sensors, ws2812) are written in cpp,
 * you can include and use them in a .ino or a .cpp file (but can NOT in .c).
 * A .ino is a .cpp indeed.
 *
 * You can define some variables in a .c file, e.g. int my_value = 1;,
 * and you can access this variable in a .ino or a .cpp by writing extern "C" int my_value;.
 *
 * So, if you want use this HomeKit library and other Arduino Libraries together,
 * 1. define your HomeKit accessory/service/characteristic in a .c file
 * 2. in your .ino, include some Arduino Libraries and you can use them normally
 *                  write extern "C" homekit_characteristic_t xxxx; to access the characteristic defined in your .c file
 *                  write your logic code (eg. read sensors) and
 *                  report your data by writing your_characteristic.value.xxxx_value = some_data; homekit_characteristic_notify(..., ...)
 * done.
 */

#include <Arduino.h>
#include <arduino_homekit_server.h>
#include "wifi_info.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "bsec.h"
//include the Arduino library for your real sensor here, e.g. <DHT.h>

#define SDA_PIN 0x2
#define SCL_PIN 0x0
#define BME680_DEFAULT_ADDRESS (0x77)

#define LOG_D(fmt, ...)   printf_P(PSTR(fmt "\n") , ##__VA_ARGS__);

#define IQA_PROXY

void setup() {
	Serial.begin(115200);
  // while(!Serial);
  // Serial.println("\n\nI2C Scanner to scan for devices on each port pair D0 to D7");
  // scanPorts();
  // Serial.println(F("\nCONNECT to BME680..."));
  Wire.begin(SDA_PIN, SCL_PIN, BME680_DEFAULT_ADDRESS);
#ifndef IQA_PROXY
  setUpSensorBME680();
#else
  setupIAQ();
#endif
	wifi_connect(); // in wifi_info.h
	my_homekit_setup();
}

void loop() {
	my_homekit_loop();
	delay(10);
}

//==============================
// Homekit setup and loop
//==============================

// access your homekit characteristics defined in my_accessory.c
extern "C" homekit_server_config_t config;
extern "C" homekit_characteristic_t cha_temperature;
extern "C" homekit_characteristic_t cha_humidity;
// extern "C" homekit_characteristic_t cha_pressure;
extern "C" homekit_characteristic_t cha_air;
extern "C" homekit_characteristic_t cha_status_fault;

#define HOMEKIT_CONTACT_SENSOR_DETECTED       0
#define HOMEKIT_CONTACT_SENSOR_NOT_DETECTED   1

#define HOMEKIT_OCCUPANCY_DETECTED                  0
#define HOMEKIT_OCCUPANCY_NOT_DETECTED              1


static uint32_t next_heap_millis = 0;
static uint32_t next_report_millis = 0;

Adafruit_BME680 bme; // I2C
Bsec iaqSensor;
String output;

void my_homekit_setup() {
	arduino_homekit_setup(&config);
}

void my_homekit_loop() {
	arduino_homekit_loop();
	const uint32_t t = millis();
	if (t > next_report_millis) {
		// report sensor values every 10 seconds
		next_report_millis = t + 3 * 1000;
		my_homekit_report();
	}
	if (t > next_heap_millis) {
		// show heap info every 5 seconds
		next_heap_millis = t + 10 * 1000;
		LOG_D("Free heap: %d, HomeKit clients: %d",
				ESP.getFreeHeap(), arduino_homekit_connected_clients_count());

	}
}

void my_homekit_report() {
	// FIXME, read your real sensors here.
#ifndef IQA_PROXY
  readSensorBME680();
	cha_temperature.value.float_value = bme.temperature;
	cha_humidity.value.float_value = bme.humidity;
	// cha_pressure.value.float_value = l;
	// cha_gas.value.uint8_value = c;
#else
  readIAQ();
#endif
	homekit_characteristic_notify(&cha_temperature, cha_temperature.value);
	homekit_characteristic_notify(&cha_humidity, cha_humidity.value);
	homekit_characteristic_notify(&cha_status_fault, cha_status_fault.value);
	// homekit_characteristic_notify(&cha_pressure, cha_pressure.value);
	homekit_characteristic_notify(&cha_air, cha_air.value);
}



/*#define BME_SCK 14
#define BME_MISO 12
#define BME_MOSI 13
#define BME_CS 15*/

#define SEALEVELPRESSURE_HPA (1013.25)

//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

void setUpSensorBME680() {
  // Serial.begin(115200);
  // while (!Serial);
  Serial.println(F("BME680 async test\n"));

  if (!bme.begin(BME680_DEFAULT_ADDRESS)) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!\n"));
    cha_status_fault.value.bool_value = 1;
    return;
  }
  
  cha_status_fault.value.bool_value = 0;
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

uint count = 0;
void readSensorBME680() {
    // Tell BME680 to begin measurement.
  // Serial.begin(115200);

  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    if (count % 100000 == 0) {
      // Serial.println(F("Failed to begin reading :("));
      count = 1;

    }
    count++;
    return;
  }
  Serial.print(F("Reading started at "));
  Serial.print(millis());
  Serial.print(F(" and will finish at "));
  Serial.println(endTime);

  Serial.println(F("You can do other work during BME680 measurement."));
  delay(50); // This represents parallel work.
  // There's no need to delay() until millis() >= endTime: bme.endReading()
  // takes care of that. It's okay for parallel work to take longer than
  // BME680's measurement time.

  // Obtain measurement results from BME680. Note that this operation isn't
  // instantaneous even if milli() >= endTime due to I2C/SPI latency.
  if (!bme.endReading()) {
    if (count % 100000 == 0) {
      Serial.println(F("Failed to complete reading :("));
      count = 1;
    }
    count++;
    return;
  }
  Serial.print(F("Reading completed at "));
  Serial.println(millis());

  Serial.print(F("Temperature = "));
  Serial.print(bme.temperature);
  Serial.println(F(" *C"));

  Serial.print(F("Pressure = "));
  Serial.print(bme.pressure / 100.0);
  Serial.println(F(" hPa"));

  Serial.print(F("Humidity = "));
  Serial.print(bme.humidity);
  Serial.println(F(" %"));

  Serial.print(F("Gas = "));
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(F(" KOhms"));

  Serial.print(F("Approx. Altitude = "));
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(F(" m"));

  Serial.println();
}


void setupIAQ()
{
  iaqSensor.begin(BME680_DEFAULT_ADDRESS, Wire);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();
  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();
  // Print the header
  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
  Serial.println(output);
}

void readIAQ() {
  unsigned long time_trigger = millis();
  if (! iaqSensor.run()) { // If no data is available
    checkIaqSensorStatus();
    return;
  }

  cha_temperature.value.float_value = iaqSensor.temperature;
	cha_humidity.value.float_value = iaqSensor.humidity;

  uint8_t iaqRangeMap[6][2] = {
    {0, 19},
    {20, 40}, // 1
    {40, 60}, // 2
    {60, 80}, // 3
    {80, 92}, // 4
    {92, 100} // 5
  };
  uint8_t iaq = 0;
  for (uint8_t i = 0; i < 6; ++i) {
    if (iaqSensor.staticIaq >= iaqRangeMap[i][0] && iaqSensor.staticIaq < iaqRangeMap[i][1]) {
      iaq = i;
      break;
    }
  }
  cha_air.value.uint8_value = iaq;
  
  output = String(time_trigger);
  output += ", " + String(iaqSensor.rawTemperature);
  output += ", " + String(iaqSensor.pressure);
  output += ", " + String(iaqSensor.rawHumidity);
  output += ", " + String(iaqSensor.gasResistance);
  output += ", " + String(iaqSensor.iaq);
  output += ", " + String(iaqSensor.iaqAccuracy);
  output += ", " + String(iaqSensor.temperature);
  output += ", " + String(iaqSensor.humidity);
  output += ", " + String(iaqSensor.staticIaq);
  output += ", " + String(iaqSensor.co2Equivalent);
  output += ", " + String(iaqSensor.breathVocEquivalent);
  Serial.println(output);

  Serial.print("Temperature = "); Serial.print(iaqSensor.temperature); Serial.println(" *C");
  Serial.print("Pressure = "); Serial.print(iaqSensor.pressure / 100.0); Serial.println(" hPa");
  Serial.print("Humidity = "); Serial.print(iaqSensor.humidity); Serial.println(" %");
  Serial.print("IAQ = "); Serial.print(iaqSensor.staticIaq); Serial.println("");
  Serial.print("CO2 equiv = "); Serial.print(iaqSensor.co2Equivalent); Serial.println("");
  Serial.print("Breath VOC = "); Serial.print(iaqSensor.breathVocEquivalent); Serial.println("");
  Serial.println();
  delay(2000);
}
// Helper function definitions
void checkIaqSensorStatus(void)
{
  if (iaqSensor.bsecStatus != BSEC_OK) {
    cha_status_fault.value.bool_value = 1;
    if (iaqSensor.bsecStatus < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.bsecStatus);
      Serial.println(output);
      for (;;)  delay(10);
    } else {
      output = "BSEC warning code : " + String(iaqSensor.bsecStatus);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme68xStatus != BME68X_OK) {
    cha_status_fault.value.bool_value = 1;
    if (iaqSensor.bme68xStatus < BME68X_OK) {
      output = "BME68X error code : " + String(iaqSensor.bme68xStatus);
      Serial.println(output);
      for (;;)  delay(10);
    } else {
      output = "BME68X warning code : " + String(iaqSensor.bme68xStatus);
      Serial.println(output);
    }
  }
}
