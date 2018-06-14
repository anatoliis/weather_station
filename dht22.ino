#include <SimpleDHT.h>

#define DHT_PIN D5

SimpleDHT22 dht;

void dht22_fetchData() {
  float newTemperature = 0;
  float newHumidity = 0;
  int error = SimpleDHTErrSuccess;
  if ((error = dht.read2(DHT_PIN, &newTemperature, &newHumidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT22 failed, err=");
    Serial.println(error);
  } else {
    temperature_dht = newTemperature;
  }
}

