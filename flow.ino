#define FLOW_SENSOR_PIN D3
#define FLOW_SENSOR_INTERRUPT digitalPinToInterrupt(FLOW_SENSOR_PIN)

float flowCalibrationFactor = 4.5;

volatile int flowPulseCount;
unsigned int flowMillilitres;
unsigned long lastFlowUpdateTime;

void flow_initialize() {
  pinMode(FLOW_SENSOR_PIN, INPUT);
  digitalWrite(FLOW_SENSOR_PIN, HIGH);

  flowPulseCount = 0;
  flow_rate = 0.0;
  flowMillilitres = 0;
  totalMillilitres = 0;
  millilitres = 0;
  lastFlowUpdateTime = 0;
  attachInterrupt(FLOW_SENSOR_INTERRUPT, flow_pulseCounter, FALLING);
}

void flow_pulseCounter() {
  flowPulseCount++;
  Serial.print("flow: ");
  Serial.println(flowPulseCount);
}

void flow_processCounter() {
  if((unsigned long)(millis() - lastFlowUpdateTime) > 1000)
  {
    detachInterrupt(FLOW_SENSOR_INTERRUPT);
    // TODO: implement saving flow_rate
    flow_rate = ((1000.0 / (unsigned long)(millis() - lastFlowUpdateTime)) * flowPulseCount) / flowCalibrationFactor;
    Serial.println("-----");
    Serial.print("flow rate: ");
    Serial.println(flow_rate);
    lastFlowUpdateTime = millis();
    flowMillilitres = (flow_rate / 60) * 1000;
    Serial.print("flow ml: ");
    Serial.println(flowMillilitres);
    totalMillilitres += flowMillilitres;
    millilitres += flowMillilitres;
    Serial.print("total ml: ");
    Serial.println(totalMillilitres);
    Serial.println(millilitres);
    flowPulseCount = 0;
    attachInterrupt(FLOW_SENSOR_INTERRUPT, flow_pulseCounter, FALLING);
  }
}

