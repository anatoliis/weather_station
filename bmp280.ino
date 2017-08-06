#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

bool bmpSensorFound = false;

Adafruit_BMP280 bme; // using hardware i2c bus

void bmp280_initialize() {
  bmpSensorFound = bme.begin();
  if (!bmpSensorFound) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  }
}

void bmp280_fetchPressure() {
  if (bmpSensorFound) {
    temperature_bmp = bme.readTemperature();
    pressure = bme.readPressure();  
  }
}

