#ifndef __CONFIG_H__
#define __CONFIG_H__

//#define DEBUG

//#define SENSOR_BME280
#define SENSOR_HTU21

#define TPL_DONE_PIN A0

#define EEPROM_NODE_ADDRESS 0
#define EEPROM_CHANNEL_ADDRESS 1
#define EEPROM_SPEED_ADDRESS 2
#define EEPROM_POWER_ADDRESS 3
#define EEPROM_ENABLE_STATUS_LED_ADDRESS 4

#define PROG_PIN 2
#define RADIO_CE 9
#define RADIO_CSN 10
#define CONFIG_LED A1
#define STATUS_LED A2

#define BATTERY_MIN_VOLTAGE 2500
#define BATTERY_MAX_VOLTAGE 3600

#ifdef SENSOR_BME280
#include "./src/BME280.h"
#endif

#ifdef SENSOR_HTU21
#include "SparkFunHTU21D.h"
#endif

#endif