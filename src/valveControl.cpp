#include <Arduino.h>
#include "valveControl.h"
#include "adjustableSettings.h"

const uint8_t ANALOG_PRESSURE_RANGE = ANALOG_PRESSURE_MAX - ANALOG_PRESSURE_MIN;
const uint8_t ANALOG_VENT_RANGE = ANALOG_VENT_MAX - ANALOG_VENT_MIN;

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
}

// analog write to both valves
void pressurizeProportional(float controlSignal){
  writeVentValve(0);
  float temp = mapFloat(controlSignal, 0.0, 1.0, 0.0, ANALOG_PRESSURE_RANGE);
  int setpoint = constrain(temp+ANALOG_PRESSURE_MIN, 0, ANALOG_PRESSURE_MAX);
  writePressureValve(setpoint);
  
}

void ventProportional(float controlSignal){
  writePressureValve(0);
  float temp = mapFloat(controlSignal, 0.0, 1.0, 0.0, ANALOG_VENT_RANGE);
  int setpoint = constrain(temp+ANALOG_VENT_MIN, 0, ANALOG_VENT_MAX);
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
