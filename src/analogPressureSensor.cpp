#include <Arduino.h>
#include "analogPressureSensor.h"

// Define the sensor pin
const int SENSOR_PIN = A7;
double SensorPressure;
const double filterAlpha = 0.1;

void getSensorPressure() {
  int SensorReading = analogRead(SENSOR_PIN);
  // Conversion equation between pressure sensor's output range in analog values
  // to volts (min 0.5V, max 4.5V, range of 4.0V), and then from volts to PSI.
  // P = ((5V * SensorReading) / (MaxAnalogSignalValue) - 0.5V) * 150PSI/4V
  SensorPressure = (5.0 * SensorReading / 1023.0 - 0.5) * 37.5; // Analog to PSI
}

void getFilteredSensorPressure() {
  // Filter the sensor reading with simple low-pass filter
  //previousPressure = SensorPressure;
  getSensorPressure();
  SensorPressure = filterAlpha * SensorPressure + (1.0 - filterAlpha) * SensorPressure; //replace
}

void displayPressure() {
  // You can replace this with any LCD or other display code if needed
}
