#include <Arduino.h>
#include "analogPressureSensor.h"
#include "adjustableSettings.h"

// Define the sensor pin
double SensorPressure;
double previousPressure;

void getSensorPressure() {
  int SensorReading = analogRead(SENSOR_PIN);
  // Conversion equation between pressure sensor's output range in analog values
  // to volts (min 0.5V, max 4.5V, range of 4.0V), and then from volts to PSI.
  // P = ((5V * SensorReading) / (MaxAnalogSignalValue) - 0.5V) * 150PSI/4V
  SensorPressure = (5.0 * SensorReading / 1023.0 - 0.5) * 37.5; // Analog to PSI
}

void UpdateFilteredSensorPressure() {
  // simple estimated moving average filter to reduce signal noise
  previousPressure = SensorPressure;
  getSensorPressure();
  SensorPressure = (1.0 - FILTER_ALPHA) * SensorPressure + FILTER_ALPHA  * previousPressure;
}

void displayPressure() {
  // You can replace this with any LCD or other display code if needed
}
