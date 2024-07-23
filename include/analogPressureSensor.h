#ifndef ANALOG_PRESSURE_SENSOR_H
#define ANALOG_PRESSURE_SENSOR_H

extern const int SENSOR_PIN;
extern double SensorPressure;

void getSensorPressure();
void displayPressure();
void UpdateFilteredSensorPressure();

#endif
