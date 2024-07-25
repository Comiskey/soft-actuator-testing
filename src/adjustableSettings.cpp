#include <Arduino.h>
#include "adjustableSettings.h"

// Start and stop button pins
 const int START_BUTTON_PIN = A5; 
 const int STOP_BUTTON_PIN = A4;

// Pnuematic valve pins
 const int PRESSURE_PIN = 6;
 const int VENT_PIN = 5;

// Pnuematic valve analog write values
const int ANALOG_PRESSURE_MIN = 146;
const int ANALOG_PRESSURE_MAX = 170;
const int ANALOG_VENT_MIN = 156;
const int ANALOG_VENT_MAX = 180;

// Frequency variables
const int PRESSURE_READ_DELAY = 10; // milliseconds aka 100Hz
const int INTERP_CALC_DELAY = 50; // milliseconds aka 20Hz
const int CONTROLLER_DELAY = 50; // milliseconds aka 20Hz

// Pressure sensor variables
const int SENSOR_PIN = A7;
const double FILTER_ALPHA = 0.0;
const int PRESSURE_MAX = 30; // PSI

// PID Controller variables
const double THRESHOLD = 2; // PSI
const int OUTPUT_MIN = -1.0;
const int OUTPUT_MAX = 1.0;
const double KP = 0.1; // Clark's defaults: Kp = 0.1, Ki = 0.001, Kd = 0.0. Good starting point
const double KI = 0.001;
const double KD = 0.0;

// Trajectory variables
// WARNING: Ensure the first and last pressures of your trajectory path
// match so as to ensure smooth transitions between cycles
// Example step function trajectory
const float TIMES[] = {0, 100, 2000, 2100, 3000}; //milliseconds
const double PRESSURES[] = {0, 15, 15, 0, 0}; // PSI
// Example sinusoidal trajectory
// const float times[] = {0, 1111, 2222, 3333, 4444, 5555, 6666, 7777, 8888, 10000}; // milliseconds
// const double pressures[] = {
//     0, 
//     20 * sin(M_PI / 9 * 1), 
//     20 * sin(M_PI / 9 * 2), 
//     20 * sin(M_PI / 9 * 3), 
//     20 * sin(M_PI / 9 * 4), 
//     20 * sin(M_PI / 9 * 5), 
//     20 * sin(M_PI / 9 * 6), 
//     20 * sin(M_PI / 9 * 7), 
//     20 * sin(M_PI / 9 * 8), 
//     0 }; // PSI

// Example ramp (burst) trajectory
// TODO

// Initialize trajectory
const int TRAJ_SIZE = sizeof(TIMES) / sizeof(TIMES[0]);
