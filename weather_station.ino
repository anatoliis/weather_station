#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

String CSVHeader = "t1,t2,t3,t4,tc,t0,pr,hm,fr,ml,ts\n";

const char* ssid = "********";
const char* password = "********";
IPAddress wifi_ip(192, 168, 0, 195);
IPAddress wifi_gate(192, 168, 0, 1);
IPAddress wifi_sub(255, 255, 255, 0);

int measurementsInterval = 5000;

// Data variables
float temperature_1 = -273;
float temperature_2 = -273;
float temperature_dht = -273;
float temperature_bmp = -273;
float temperature_self = -273;
float temperature_collector = -273;
float pressure = 0;
float humidity = 0;
float flow_rate = 0;
unsigned long lastFetchTimestamp = millis();
////

unsigned long totalMillilitres;
unsigned long millilitres;

// Temporary/helping
char indexBuffer[640];
char dataBuffer[128];
char dataAllBuffer[3072];
int dataAllBufferSize = 3072;
////

// Measurements store
String measurements[36];
byte maxMeasurementsInArray = 36;
byte lastMeasurementIndex = -1;
////

ESP8266WebServer HTTP(80);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  startWiFi();
  delay(300);
  
  ds18b20_initialize();
  bmp280_initialize();
  flow_initialize();
  
  startHTTP();
}

void startWiFi() {
  Serial.println("Connecting to WiFi");

//  WiFi.persistent(false);
//  WiFi.mode(WIFI_OFF);
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
  operationalLoop(100);
  bmp280_fetchPressure();
  operationalLoop(100);
  dht22_fetchData();
  lastFetchTimestamp = millis();
  operationalLoop(100);
  saveMeasurementResults();
  unsigned long timeOfLoopPassed = millis() - loopStartMillis;
  if (timeOfLoopPassed < measurementsInterval) {
    operationalLoop(measurementsInterval - timeOfLoopPassed);
  }
}

void saveMeasurementResults() {
  String data = formatDataString();
  lastMeasurementIndex++;
  if (lastMeasurementIndex >= maxMeasurementsInArray) {
    lastMeasurementIndex = 0;
  }
  measurements[lastMeasurementIndex] = data;
}

String getLastMeasurement() {
  if (lastMeasurementIndex == -1) {
    return String("");
  }
  return CSVHeader + measurements[lastMeasurementIndex] + String("\n") + millis();
}

String getAllMeasurements() {
  String result;
  if (measurements[0].length() == 0) {
    return result;
  }
  
  for (int i = 0; i < maxMeasurementsInArray; i++) {
    if (measurements[i].length() > 0) {
      result += measurements[i] + String("\n");
    }
  }
  
  if (result.length() == 0) {
    return "";
  }
  return CSVHeader + result + millis(); 
}

unsigned long lastDiag = millis();

void operationalLoop(int delayMs) {
  unsigned long start = millis();
  while (millis() - start < delayMs) {
    HTTP.handleClient();
    flow_processCounter();
    delay(1);
    if (millis() - lastDiag > 5000) {
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
  
  HTTP.on("/data_all", HTTP_GET, [](){
    Serial.println("<= GET '/data_all'");
    getAllMeasurements().toCharArray(dataAllBuffer, dataAllBufferSize);
    HTTP.send(200, "text/plain", dataAllBuffer);
  });
  
  HTTP.on("/update_resolution", HTTP_GET, [](){
    ds18b20_changeSensorsResolution();
    HTTP.send(200, "text/plain", "Done. DS18B20 resolution is 11 bits.");
  });
  
  HTTP.on("/fast_mode_on", HTTP_GET, [](){
    measurementsInterval = 1500;
    HTTP.send(200, "text/plain", "Done.");
  });
  
  HTTP.on("/fast_mode_off", HTTP_GET, [](){
    measurementsInterval = 5000;
    HTTP.send(200, "text/plain", "Done.");
  });
  
  HTTP.on("/help", HTTP_GET, [](){
    HTTP.send(200, "text/plain", "Help page.");
  });
  
  HTTP.on("/restart", HTTP_GET, [](){
    HTTP.send(200, "text/plain", "Restarting.");
    //restart
  });
  
  HTTP.begin();
}

String formatReadableResponse() {
  float averageTemperature = (temperature_1 + temperature_2 + temperature_dht) / 3;
  return \
    String("<html><head><title>Weather Station</title></head><body>") + \
    String("Temperature 1: ") + String(temperature_1, 2) + \
    String("<br>Temperature 2: ") + String(temperature_2, 2) + \
    String("<br>Temperature 3: ") + String(temperature_dht, 2) + \
    String("<br>Temperature 4: ") + String(temperature_bmp, 2) + \
    String("<br>Collector temperature: ") + String(temperature_collector, 2) + \
    String("<br>Average temperature: ") + String(averageTemperature, 2) + \
    String("<br>Pressure: ") + String(pressure, 2) + String(" / ") + String(pressure / 133.3223684, 2) + \
    String("<br>Humidity: ") + String(humidity, 2) + \
    String("<br>Unit temperature: ") + String(temperature_self, 2) + \
    String("<br>Flow rate: ") + String(flow_rate, 2) + \
    String("<br>Water millilitres: ") + String(totalMillilitres) + \
    String("<br>Measured: ") + String((float)(unsigned long)(millis() - lastFetchTimestamp) / 1000, 2) + String(" sec ago") + \
    String("<br><br><small><a href=\"/data\">Raw data</a></small>") + \
    String("<br><small><a href=\"/data_all\">Last 36 measurements</a></small>") + \
    String("<br><br><small><a href=\"/help\">Help</a></small>") + \
    String("</body></html>");
}

String formatDataString() {
  unsigned long ml = millilitres;
  millilitres = 0;

  return String(temperature_1, 2) + \
    String(",") + String(temperature_2, 2) + \
    String(",") + String(temperature_dht, 2) + \
    String(",") + String(temperature_bmp, 2) + \
    String(",") + String(temperature_collector, 2) + \
    String(",") + String(temperature_self, 2) + \
    String(",") + String(pressure, 2) + \
    String(",") + String(humidity, 2) + \
    String(",") + String(flow_rate, 2) + \
    String(",") + String(ml, DEC) + \
    String(",") + String(lastFetchTimestamp, DEC);
}

