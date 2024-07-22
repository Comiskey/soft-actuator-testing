#ifndef VALVE_CONTROL_H
#define VALVE_CONTROL_H

extern const int PRESSURE_PIN;
extern const int VENT_PIN;

// Pressure valve control functions
void writePressureValve(int pwmValue);
void openPressureValve();
void closePressureValve();

// Vent valve control functions
void writeVentValve(int pwmValue);
void openVentValve();
void closeVentValve();

// simultaneous valve control functions
void closeValves();
void pressurize();
void vent();

// analog valve control functions
void pressurizeProportional(float controlSignal);
void ventProportional(float controlSignal);
void sendSignalToValves(float controlSignal);

#endif
