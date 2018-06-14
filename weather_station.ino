#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

String CSVHeader = "t1,t2,t3,tc,tc2,ts\n";

const char* ssid = "Tenda_3D3FF0";
const char* password = "********";
IPAddress wifi_ip(192, 168, 0, 195);
IPAddress wifi_gate(192, 168, 0, 1);
IPAddress wifi_sub(255, 255, 255, 0);

int measurementsInterval = 5000;

// Data variables
float temperature_1 = -273;
float temperature_2 = -273;
float temperature_dht = -273;
float temperature_collector = -273;
float temperature_collector2 = -273;
unsigned long lastFetchTimestamp = millis();
////

//unsigned long totalMillilitres;
//unsigned long millilitres;

// Temporary/helping
char indexBuffer[640];
char dataBuffer[128];
char dataAllBuffer[3072];
int dataAllBufferSize = 3072;
////

ESP8266WebServer HTTP(80);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  startWiFi();
  delay(300);
  
  ds18b20_initialize();
  
  startHTTP();
}

void startWiFi() {
  Serial.println("Connecting to WiFi");

  WiFi.mode(WIFI_STA);
  WiFi.config(wifi_ip, wifi_gate, wifi_sub);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi connection failed");
    while(1) delay(100);
  }
}

unsigned long loopStartMillis;

void loop() {
  loopStartMillis = millis();
  ds18b20_fetchTemperatures();
  operationalLoop(140);
  dht22_fetchData();
  lastFetchTimestamp = millis();
  operationalLoop(140);
  unsigned long timeOfLoopPassed = millis() - loopStartMillis;
  if (timeOfLoopPassed < measurementsInterval) {
    operationalLoop(measurementsInterval - timeOfLoopPassed);
  }
}

String getLastMeasurement() {
  return CSVHeader + formatDataString() + String("\n") + millis();
}

unsigned long lastDiag = millis();

void operationalLoop(int delayMs) {
  unsigned long start = millis();
  while (millis() - start < delayMs) {
    HTTP.handleClient();
    delay(1);
    if (millis() - lastDiag > 10000) {
      WiFi.printDiag(Serial);
      Serial.println("-----");
      lastDiag = millis();
    }
  }
}

void startHTTP() {
  Serial.println("Starting HTTP...");
  
  HTTP.on("/", HTTP_GET, [](){
    Serial.println("<= GET '/'");
    formatReadableResponse().toCharArray(indexBuffer, 640);
    HTTP.send(200, "text/html", indexBuffer);
  });
  
  HTTP.on("/data", HTTP_GET, [](){
    Serial.println("<= GET /data'");
    getLastMeasurement().toCharArray(dataBuffer, 128);
    HTTP.send(200, "text/plain", dataBuffer);
  });
  
  HTTP.on("/update_resolution", HTTP_GET, [](){
    ds18b20_changeSensorsResolution();
    HTTP.send(200, "text/plain", "Done. DS18B20 resolution is set to 11 bits.");
  });
  
  HTTP.on("/fast_mode_on", HTTP_GET, [](){
    measurementsInterval = 1500;
    HTTP.send(200, "text/plain", "Done.");
  });
  
  HTTP.on("/fast_mode_off", HTTP_GET, [](){
    measurementsInterval = 5000;
    HTTP.send(200, "text/plain", "Done.");
  });
  
  HTTP.begin();
}

String formatReadableResponse() {
  float averageTemperature = (temperature_1 + temperature_2 + temperature_dht) / 3;
  float averageCollectorTemperature = (temperature_collector + temperature_collector2) / 2;
  return \
    String("<html><head><title>Weather Station</title></head><body>") + \
    String("<br>Temperature: ") + String(averageTemperature, 2) + \
    String("<br>Collector temperature: ") + String(averageCollectorTemperature, 2) + \
    String("<br><br>Temperature 1: ") + String(temperature_1, 2) + \
    String("<br>Temperature 2: ") + String(temperature_2, 2) + \
    String("<br>Temperature 3: ") + String(temperature_dht, 2) + \
    String("<br>Collector temperature 1: ") + String(temperature_collector, 2) + \
    String("<br>Collector temperature 2: ") + String(temperature_collector2, 2) + \
    String("<br>Measured: ") + String((float)(unsigned long)(millis() - lastFetchTimestamp) / 1000, 2) + String(" sec ago") + \
    String("<br><br><small><a href=\"/data\">Raw data</a></small>") + \
    String("</body></html>");
}

String formatDataString() {
  return String(temperature_1, 2) + \
    String(",") + String(temperature_2, 2) + \
    String(",") + String(temperature_dht, 2) + \
    String(",") + String(temperature_collector, 2) + \
    String(",") + String(temperature_collector2, 2) + \
    String(",") + String(lastFetchTimestamp, DEC);
}

