#include <avr/eeprom.h>
#include "./src/GPIO.h"
#include "RF24.h"
#include "./src/config.h"

const char *power_names[4] = {"MIN", "LOW", "HIGH", "MAX"};
const char *speed_names[3] = {"1MBPS", "2MBPS", "250KBPS"};
const uint64_t pipes[5] = {0xF0F0F0F0D2LL, 0xF0F0F0F0C3LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0A5LL, 0xF0F0F0F096LL};

#ifdef SENSOR_BME280
BME280 bme280;
SensorCalibration calibration;
#endif

#ifdef SENSOR_HTU21
HTU21D htu21;
#endif

RF24 radio(RADIO_CE, RADIO_CSN);

uint8_t from_node;
uint8_t rf_channel;
uint8_t rf_speed;
uint8_t rf_power;
uint8_t status_led_enabled;

struct ExternalSensor
{
  signed int temperature;
  signed int humidity;
  uint8_t battery;
};

ExternalSensor data;

void setup()
{
  pinAsOutput(TPL_DONE_PIN);
  digitalLow(TPL_DONE_PIN);

  radio.begin();
  radio.powerDown();

  disableADC();
  disableSPI();
  disableTWI();

  pinAsInputPullUp(PROG_PIN);
  pinAsOutput(CONFIG_LED);
  pinAsOutput(STATUS_LED);
  digitalLow(CONFIG_LED);
  digitalLow(STATUS_LED);

  from_node = eeprom_read_byte((uint8_t *)EEPROM_NODE_ADDRESS);
  rf_channel = eeprom_read_byte((uint8_t *)EEPROM_CHANNEL_ADDRESS);
  rf_speed = eeprom_read_byte((uint8_t *)EEPROM_SPEED_ADDRESS);
  rf_power = eeprom_read_byte((uint8_t *)EEPROM_POWER_ADDRESS);
  status_led_enabled = eeprom_read_byte((uint8_t *)EEPROM_ENABLE_STATUS_LED_ADDRESS);

  bool configIsValid = (from_node >= 1 && from_node <= 5) &&
                       (rf_channel >= 1 && rf_channel <= 125) &&
                       (rf_speed >= 0 && rf_speed <= 2) &&
                       (rf_power >= 0 && rf_speed <= 3);

#ifdef DEBUG
  Serial.begin(9600);
#endif

  if (!configIsValid)
  {
    from_node = 1;
    rf_channel = 80;
    rf_speed = 2;
    rf_power = 3;
    status_led_enabled = 1;
  }

  if (isLow(PROG_PIN))
  {
    Serial.begin(9600);

    Serial.println(F("\nConfiguration mode"));

    digitalHigh(CONFIG_LED);
    configure();
    digitalLow(CONFIG_LED);
  }
  else
  {
#ifdef DEBUG
    printConfig();
#endif
  }

#ifdef DEBUG
  Serial.println(F("Start"));
  unsigned long startTime = millis();
#endif

  data.temperature = 0;
  data.humidity = 0;

  enableADC();
  data.battery = getBatteryPercent();
  disableADC();

  enableStatusLED();
  enableTWI();

  //delay(2);

#ifdef SENSOR_BME280
  if (bme280.begin(0x76, &calibration) == false)
  {
#ifdef DEBUG
    Serial.println(F("Sensor connect failed"));
#endif

    warningBlink(200);
    powerDown();
  }
  else
  {
    bme280.setFilter(1);
    bme280.setTempOverSample(2);
    bme280.setHumidityOverSample(2);
    bme280.setMode(MODE_NORMAL);

    uint8_t measurementCount = 0;
    while (bme280.isMeasuring() && measurementCount <= 10)
    {
      delay(1);
      measurementCount++;
    };

    data.temperature = bme280.readTempC() * 100;
    data.humidity = bme280.readFloatHumidity() * 100;
  }
#endif

#ifdef SENSOR_HTU21
  htu21.begin();

  data.temperature = htu21.readTemperature() * 100;
  data.humidity = htu21.readHumidity() * 100;
#endif

  disableTWI();
  enableSPI();

  radio.powerUp();
  radio.stopListening();
  radio.setAutoAck(false);
  radio.setChannel(rf_channel);
  radio.setPayloadSize(sizeof(data));
  radio.setPALevel(rf24_pa_dbm_e(rf_power));
  radio.setDataRate(rf24_datarate_e(rf_speed));
  radio.openWritingPipe(pipes[from_node - 1]);

  radio.write(&data, sizeof(data));
  radio.powerDown();

  disableStatusLED();

#ifdef DEBUG
  Serial.print(F("Hum: "));
  Serial.print(data.humidity);
  Serial.println();

  Serial.print(F("Temp: "));
  Serial.print(data.temperature);
  Serial.println();

  Serial.print(F("Bat: "));
  Serial.print(data.battery);
  Serial.print('%');
  Serial.println();

  Serial.print(F("Running time(ms): "));
  Serial.println(millis() - startTime);
  Serial.println(F("Sleep"));
  delay(1000); // Delay for complete Serial write
#endif

  powerDown();
}

void loop()
{
}

void warningBlink(uint16_t delayMs)
{
  for (uint8_t i = 0; i <= 3; i++)
  {
    enableStatusLED();
    delay(delayMs);
    disableStatusLED();
    delay(delayMs);
  }

  disableStatusLED();
}

void configure()
{
  while (!Serial)
  {
    ; // wait for serial port to connect.
  }

  Serial.flush();

  while (1)
  {
    Serial.println(F("*** Node address (1-5):"));
    while (!Serial.available())
      ;

    from_node = Serial.readStringUntil('\n').toInt();

    if (from_node >= 1 && from_node <= 5)
    {
      eeprom_write_byte((uint8_t *)EEPROM_NODE_ADDRESS, from_node);
      Serial.println(from_node);
      break;
    }
    else
    {
      Serial.println(F("Wrong value!"));
    }
  }

  while (1)
  {
    Serial.println(F("*** Node channel id (1-125):"));
    while (!Serial.available())
      ;

    rf_channel = Serial.readStringUntil('\n').toInt();

    if (rf_channel >= 1 && rf_channel <= 125)
    {
      eeprom_write_byte((uint8_t *)EEPROM_CHANNEL_ADDRESS, rf_channel);
      Serial.println(rf_channel);
      break;
    }
    else
    {
      Serial.println(F("Wrong value!"));
    }
  }

  while (1)
  {
    Serial.println(F("*** Radio speed (0-2):\n0 - 1MBPS\r\n1 - 2MBPS\r\n2 - 250KBPS"));
    while (!Serial.available())
      ;

    rf_speed = Serial.readStringUntil('\n').toInt();

    if (rf_speed >= 0 && rf_speed <= 2)
    {
      eeprom_write_byte((uint8_t *)EEPROM_SPEED_ADDRESS, rf_speed);
      Serial.println(rf_speed);
      break;
    }
    else
    {
      Serial.println(F("Wrong value!"));
    }
  }

  while (1)
  {
    Serial.println(F("*** Radio power (0-3):"));
    Serial.println(F("0 - MIN\r\n1 - LOW\r\n2 - HIGH\r\n3 - MAX"));
    while (!Serial.available())
      ;

    rf_power = Serial.readStringUntil('\n').toInt();

    if (rf_power >= 0 && rf_speed <= 3)
    {
      eeprom_write_byte((uint8_t *)EEPROM_POWER_ADDRESS, rf_power);
      Serial.println(rf_power);
      break;
    }
    else
    {
      Serial.println(F("Wrong value!"));
    }
  }

  while (1)
  {
    Serial.println(F("*** Enable status LED?"));
    Serial.println(F("1 - Enable\r\n0 - Disable"));
    while (!Serial.available())
      ;

    status_led_enabled = Serial.readStringUntil('\n').toInt();

    if (status_led_enabled >= 0 && status_led_enabled < 2)
    {
      eeprom_write_byte((uint8_t *)EEPROM_ENABLE_STATUS_LED_ADDRESS, status_led_enabled);
      Serial.println((status_led_enabled == 1) ? F("Enabled") : F("Disabled"));
      break;
    }
    else
    {
      Serial.println(F("Wrong value!"));
    }
  }

  printConfig();
}

void printConfig()
{
  Serial.println();
  Serial.print(F("ID:"));
  Serial.print(from_node);
  Serial.println();
  Serial.print(F("Channel:"));
  Serial.print(rf_channel);
  Serial.println();
  Serial.print(F("Speed:"));
  Serial.print(speed_names[rf_speed]);
  Serial.println();
  Serial.print(F("Power:"));
  Serial.print(power_names[rf_power]);
  Serial.println();
  delay(1000);
}

void enableStatusLED()
{
  if (status_led_enabled)
  {
    digitalHigh(STATUS_LED);
  }
}

void disableStatusLED()
{
  if (status_led_enabled)
  {
    digitalLow(STATUS_LED);
  }
}

uint16_t readVcc()
{
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC))
    ;

  uint8_t low = ADCL;
  uint8_t high = ADCH;

  uint16_t result = (high << 8) | low;

  return 1108352L / result;
}

void disableADC()
{
  ADCSRA = 0;
  power_adc_disable();
}

void enableADC()
{
  power_adc_enable();
  ADCSRA |= 1 << ADEN;
}

void enableSPI()
{
  power_spi_enable();
  SPCR |= 1 << SPE;
}

void disableSPI()
{
  SPCR &= ~_BV(SPE);
  power_spi_disable();
}

void enableTWI()
{
  power_twi_enable();
}

void disableTWI()
{
  power_twi_disable();
  TWCR &= ~(_BV(TWEN) | _BV(TWIE) | _BV(TWEA));
  digitalLow(SDA);
  digitalLow(SCL);
}

uint8_t getBatteryPercent()
{
  float batteryV = readVcc();
  int batteryPcnt = (((batteryV - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100);

  return constrain(batteryPcnt, 0, 100);
}

void powerDown()
{
  while (1)
  {
    digitalHigh(TPL_DONE_PIN);
    delayMicroseconds(1);
    digitalLow(TPL_DONE_PIN);
    delayMicroseconds(1);
  }
}