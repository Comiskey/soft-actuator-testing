#include <Arduino.h>
#include "valveControl.h"

const int PRESSURE_PIN = 6;
const int VENT_PIN = 5;

uint8_t analogPressureMin = 146;
uint8_t analogVentMin = 156;

uint8_t analogPressureMax = 170;
uint8_t analogVentMax = 180;

uint8_t analogPressureRange = analogPressureMax - analogPressureMin;
uint8_t analogVentRange = analogVentMax - analogVentMin;

// helper mapping function
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Pressure valve control functions
void writePressureValve(int pwmValue) {
  analogWrite(PRESSURE_PIN, pwmValue);
}

void openPressureValve() {
  digitalWrite(PRESSURE_PIN, HIGH);
}

void closePressureValve() {
  digitalWrite(PRESSURE_PIN, LOW);
}

// Vent valve control functions
void writeVentValve(int pwmValue) {
  analogWrite(VENT_PIN, pwmValue);
}

void openVentValve() {
  digitalWrite(VENT_PIN, HIGH);
}

void closeVentValve() {
  digitalWrite(VENT_PIN, LOW);
}

// simultaneous valve control functions
void closeValves() {
  closePressureValve();
  closeVentValve();
}

void pressurize(){
  openPressureValve();
  closeVentValve();
}

void vent(){
  openVentValve();
  closePressureValve();
}

// analog write to both valves
void pressurizeProportional(float controlSignal){
  writeVentValve(0);
  float temp = mapFloat(controlSignal, 0.0, 1.0, 0.0, analogPressureRange);
  int setpoint = constrain(temp+analogPressureMin, 0, analogPressureMax);
  writePressureValve(setpoint);
  
}

void ventProportional(float controlSignal){
  writePressureValve(0);
  float temp = mapFloat(controlSignal, 0.0, 1.0, 0.0, analogVentRange);
  int setpoint = constrain(temp+analogVentMin, 0, analogVentMax);
  writeVentValve(setpoint);
}

void sendSignalToValves(float controlSignal){
  if (controlSignal < 0.0){
    ventProportional(abs(controlSignal));
  }
  else if(controlSignal > 0.0){
    pressurizeProportional(abs(controlSignal));
  }
  else{
    closeValves();
  }
}
