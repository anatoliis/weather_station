#include <OneWire.h>

#define ONE_WIRE_BUS D6 // DS18B20 array

byte temperatureSensor1[8] = {0x28, 0xff, 0x70, 0xf3, 0x87, 0x16, 0x03, 0x60};
byte temperatureSensor2[8] = {0x28, 0xff, 0x34, 0xff, 0xc0, 0x16, 0x05, 0x12};
byte temperatureSensorCollector[8];
byte temperatureSensorCollector2[8];

OneWire oneWire(ONE_WIRE_BUS);

void ds18b20_initialize() {
  ds18b20_findCollectorSensorAddress();
}

void ds18b20_findCollectorSensorAddress() {
  byte newSensorAddress[8];
  bool firstFound = false;
  bool secondFound = false;
  
  while (true) {
    bool found = oneWire.search(newSensorAddress);
    bool valid = ds18b20_validateSensorAddress(temperatureSensorCollector);
    bool isOk = valid && ds18b20_isNewAddress(newSensorAddress);
    if (!found) {
      Serial.println("No collector's DS18B20 found");
      break;
    }
    if (isOk) {
      if (!firstFound) {
        memcpy(temperatureSensorCollector, newSensorAddress, 8);
        firstFound = true;
      } else {
        memcpy(temperatureSensorCollector2, newSensorAddress, 8);
      }
      Serial.print("Found DS18B20: ");
      ds18b20_printAddress(newSensorAddress);
    }
  }
}

void ds18b20_printAddress(byte* address)
{
  bool isFirst = true;
  for (uint8_t i = 0; i < 8; ++i)
  {
    if (!isFirst) {
      Serial.print("|");
    } else {
      isFirst = false;
    }
    Serial.print(address[i], HEX);
  }
  Serial.println();
}

bool ds18b20_isNewAddress(byte* address) {
  bool isNew = !ds18b20_compareAddresses(address, temperatureSensor1);
  if (!isNew) return false;
  isNew = !ds18b20_compareAddresses(address, temperatureSensor2);
  if (!isNew) return false;
  isNew = !ds18b20_compareAddresses(address, temperatureSensorCollector);
  return isNew;
}

bool ds18b20_compareAddresses(byte* first, byte* second) {
  for (int i = 0; i < 8; i++) {
    if (first[i] != second[i]) {
      return false;
    }
  }
  return true;
}

bool ds18b20_validateSensorAddress(byte* address) {
  return OneWire::crc8(address, 7) == address[7];
}

void ds18b20_fetchTemperatures() {
  ds18b20_startConversion(temperatureSensor1);
  ds18b20_startConversion(temperatureSensor2);
  ds18b20_startConversion(temperatureSensorCollector);
  ds18b20_startConversion(temperatureSensorCollector2);
  
  operationalLoop(800);
  
  temperature_1 = ds18b20_readTemperature(temperatureSensor1);
  temperature_2 = ds18b20_readTemperature(temperatureSensor2);
  temperature_collector = ds18b20_readTemperature(temperatureSensorCollector);
  temperature_collector2 = ds18b20_readTemperature(temperatureSensorCollector2);
}

void ds18b20_startConversion(byte* address) {
  const byte STARTCONVO = 0x44;
  oneWire.reset();
  oneWire.select(address);
  oneWire.write(STARTCONVO, 1);
}

float ds18b20_readTemperature(byte* address) {
  byte sensorData[12];
  bool success = ds18b20_readScratch(address, sensorData);
  if (!success) {
    return -273;
  }

  int16_t raw = (sensorData[1] << 8) | sensorData[0];
  byte cfg = (sensorData[4] & 0x60);
  if (cfg == 0x00) raw = raw & ~7;
  else if (cfg == 0x20) raw = raw & ~3;
  else if (cfg == 0x40) raw = raw & ~1;

  return (float)raw / 16.0;
}

bool ds18b20_readScratch(byte* address, byte* scratch) {
  const byte READSCRATCH = 0xBE;
  const byte SCRATCHPAD_CRC = 8;
  
  if (!oneWire.reset()) return false;
  oneWire.select(address);
  oneWire.write(READSCRATCH);
  
  for (byte i = 0; i < 9; i++) {
    scratch[i] = oneWire.read();
  }

  byte crc = oneWire.crc8(scratch, 8);
  return oneWire.reset() && (crc == scratch[SCRATCHPAD_CRC]);
}

void ds18b20_changeSensorsResolution() {
  ds18b20_changeResolution(temperatureSensor1);
  ds18b20_changeResolution(temperatureSensor2);
  ds18b20_changeResolution(temperatureSensorCollector);
  ds18b20_changeResolution(temperatureSensorCollector2);
}

void ds18b20_changeResolution(byte* address) {
  oneWire.reset();
  oneWire.select(address);
  oneWire.write(0x4E);
  oneWire.write(0x00);
  oneWire.write(0x00);
  oneWire.write(0x7f); // 0x1f - 9 bit, 0x3f - 10 bit, 0x5f - 11 bit, 0x7f - 12 bit
  oneWire.reset();

  oneWire.select(address);
  oneWire.write(0x48);
  delay(15);
}

