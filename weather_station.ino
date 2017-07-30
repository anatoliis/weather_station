#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DHT.h>;
#include <Wire.h>
#include <OneWire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define DHT_PIN D5
#define DHT_TYPE DHT22
#define ONE_WIRE_BUS D6 // DS18B20 array

byte temperatureSensor1[8] = {0x28, 0xff, 0x70, 0xf3, 0x87, 0x16, 0x03, 0x60};
byte temperatureSensor2[8] = {0x28, 0xff, 0x34, 0xff, 0xc0, 0x16, 0x05, 0x12};
byte temperatureSensorUnit[8] = {0x28, 0xFF, 0xF7, 0x61, 0xB5, 0x16, 0x03, 0x8D};
byte temperatureSensorCollector[8];

const char* ssid = "*****";
const char* password = "*****";

char responseBuffer[448];
int responseBufferSize = 448;

float temperature_1 = -273;
float temperature_2 = -273;
float temperature_dht = -273;
float temperature_bmp = -273;
float temperature_self = -273;
float temperature_collector = -273;
float pressure = 0;
float humidity = 0;
unsigned long lastFetchTimestamp = millis();

long totalMeasures = 0;
long readingErrors = 0;
bool bmpSensorFound = false;

OneWire oneWire(ONE_WIRE_BUS);
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_BMP280 bme; // using hardware i2c bus
ESP8266WebServer HTTP(80);

void setup() {
  Serial.begin(115200);
  Serial.println();

  ds18b20_initialize();
  bmp280_initialize();
  dht.begin();
  
  startWiFi();
  startHTTP();
}

void loop() {
  ds18b20_fetchTemperatures();
  bmp280_fetchPressure();
  dht22_fetchData();
  lastFetchTimestamp = millis();
  totalMeasures++;
  printResults();
  operationalLoop(5000);
}

void printResults() {
  return;
  Serial.println("--------------");
  Serial.print("temp 1: ");
  Serial.println(temperature_1);
  Serial.print("temp 2: ");
  Serial.println(temperature_2);
  Serial.print("temp self: ");
  Serial.println(temperature_self);
  Serial.print("temp dht: ");
  Serial.println(temperature_dht);
  Serial.print("temp bmp: ");
  Serial.println(temperature_bmp);
  Serial.print("humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("pressure: ");
  Serial.println(pressure);
  Serial.println("--------------");
}

void operationalLoop(int delayMs) {
  unsigned long start = millis();
  while ((unsigned long)(millis() - start) < delayMs) {
    HTTP.handleClient();
    delay(1);
  }
}

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

void dht22_fetchData() {
  float newHumidity = dht.readHumidity();
  float newTemperature = dht.readTemperature();

  if (isnan(newHumidity) || isnan(newTemperature)) {
    readingErrors++;
    Serial.println("DHT: Error reading data");
  } else {
    humidity = newHumidity;
    temperature_dht = newTemperature;
  }
}

void startWiFi() {
  Serial.println("Starting WiFi...");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi connection failed");
    while(1) delay(100);
  }
}

void startHTTP() {
  Serial.println("Starting HTTP...");

  HTTP.on("/", HTTP_GET, [](){
    formatReadableResponse().toCharArray(responseBuffer, responseBufferSize);
    HTTP.send(200, "text/html", responseBuffer);
  });

  HTTP.on("/data", HTTP_GET, [](){
    formatResponse().toCharArray(responseBuffer, responseBufferSize);
    HTTP.send(200, "text/plain", responseBuffer);
  });

  HTTP.on("/update_resolution", HTTP_GET, [](){
    ds18b20_changeSensorsResolution();
    HTTP.send(200, "text/plain", "Done. DS18B20 resolution is 11 bits.");
  });

  HTTP.begin();
}

String formatReadableResponse() {
  float averageTemperature = (temperature_1 + temperature_2 + temperature_dht) / 3;
  String response = String("<html><head><title>Weather Station</title></head><body>");
  response += String("Temperature 1: ") + String(temperature_1, 2);
  response += String("<br>Temperature 2: ") + String(temperature_2, 2);
  response += String("<br>Temperature 3: ") + String(temperature_dht, 2);
  response += String("<br>Temperature 4: ") + String(temperature_bmp, 2);
  response += String("<br>Collector temperature: ") + String(temperature_collector, 2);
  response += String("<br>Average temperature: ") + String(averageTemperature, 2);
  response += String("<br>Pressure: ") + String(pressure, 2) + String(" / ") + String(pressure / 133.3223684, 2);
  response += String("<br>Humidity: ") + String(humidity, 2);
  response += String("<br>Unit temperature: ") + String(temperature_self);
  response += String("<br>Measured: ") + String((float)(unsigned long)(millis() - lastFetchTimestamp) / 1000, 2) + String(" sec ago");
  response += String("<br>Total measures: ") + String(totalMeasures, DEC);
  response += String("<br>Errors: ") + String(readingErrors, DEC);
  response += String(" (") + String((float)readingErrors / totalMeasures * 100, 2) + String("%)");
  response += String("<br><br><small><a href=\"/data\">Raw data</a></small>");
  return response + String("</body></html>");
}

String formatResponse() {
  String response = String("t1:") + String(temperature_1, 2);
  response += String("|t2:") + String(temperature_2, 2);
  response += String("|tdht:") + String(temperature_dht, 2);
  response += String("|tbmp:") + String(temperature_bmp, 2);
  response += String("|tu:") + String(temperature_self, 2);
  response += String("|pr:") + String(pressure, 2);
  response += String("|hm:") + String(humidity, 2);
  return response + String("|ts:") + String((unsigned long)(millis() - lastFetchTimestamp));
}

void ds18b20_initialize() {
  ds18b20_findCollectorSensorAddress();
}

void ds18b20_findCollectorSensorAddress() {
  byte newSensorAddress[8];
  
  while (true) {
    bool found = oneWire.search(newSensorAddress);
    bool valid = ds18b20_validateSensorAddress(temperatureSensorCollector);
    bool isNew = valid && ds18b20_isNewAddress(newSensorAddress);
    if (isNew) {
      memcpy(temperatureSensorCollector, newSensorAddress, 8);
      Serial.print("Found DS18B20: ");
      ds18b20_printAddress(temperatureSensorCollector);
      break;
    }
    if (!found) {
      Serial.println("No collector's DS18B20 found");
      break;
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
  isNew = !ds18b20_compareAddresses(address, temperatureSensorUnit);
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
  ds18b20_startConversion(temperatureSensorUnit);
  ds18b20_startConversion(temperatureSensorCollector);
  
  operationalLoop(950);
  
  temperature_1 = ds18b20_readTemperature(temperatureSensor1);
  temperature_2 = ds18b20_readTemperature(temperatureSensor2);
  temperature_self = ds18b20_readTemperature(temperatureSensorUnit);
  temperature_collector = ds18b20_readTemperature(temperatureSensorCollector);
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
  ds18b20_changeResolution(temperatureSensorUnit);
  ds18b20_changeResolution(temperatureSensorCollector);
}

void ds18b20_changeResolution(byte* address) {
  oneWire.reset();
  oneWire.select(address);
  oneWire.write(0x4E);
  oneWire.write(0x00);
  oneWire.write(0x00);
  oneWire.write(0x5f); // 0x1f - 9 bit, 0x3f - 10 bit, 0x5f - 11 bit, 0x7f - 12 bit
  oneWire.reset();

  oneWire.select(address);
  oneWire.write(0x48);
  delay(15);
}

