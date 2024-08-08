#include <Arduino.h>
#include "adjustableSettings.h"

// Pin definitions
const int START_BUTTON_PIN = A5; 
const int STOP_BUTTON_PIN = A4;
const int SENSOR_PIN = A7;
const int PRESSURE_PIN = 6;
const int VENT_PIN = 5;

// Pnuematic valve analog write values
const int ANALOG_PRESSURE_MIN = 146;
const int ANALOG_PRESSURE_MAX = 170;
const int ANALOG_VENT_MIN = 156;
const int ANALOG_VENT_MAX = 180;

// Frequency variables
const double PRESSURE_READ_DELAY = 10; // milliseconds aka 100Hz
const int INTERP_CALC_DELAY = 50; // milliseconds aka 20Hz
const int CONTROLLER_DELAY = 50; // milliseconds aka 20Hz

// Pressure sensor variables
const double FILTER_ALPHA = 0.0;
const int PRESSURE_MAX = 40; // PSI

// PID Controller variables
const double THRESHOLD = 2; // PSI CAN BE AS HIGH AS 50% of setpoint
const int OUTPUT_MIN = -1.0;
const int OUTPUT_MAX = 1.0;
const double KP = 0.0001; // Clark's defaults: Kp = 0.1, Ki = 0.001, Kd = 0.0. Good starting point
const double KI = 0.0;
const double KD = 0.0;

/* Trajectory variables
WARNING: Ensure the first and last pressures of your trajectory path
match so as to ensure smooth transitions between cycles.
*/

// Hold funtion
const float TIMES[] = {0, 10000}; // milliseconds
const double PRESSURES[] = {1.45038, 1.45038}; // PSI

// Step function trajectory
const float TIMES[] = {0, 100, 2000, 2100, 3000}; //milliseconds
const double PRESSURES[] = {0, 15, 15, 0, 0}; // PSI

// Triangle trajectory
// const float TIMES[] = {0, 1500, 3000}; //milliseconds
// const double PRESSURES[] = {0, 20, 0}; // PSI

// Sawtooth trajectory
// ADDING DELAY TO TEMP FIX CYCLE TRACKING ISSUE\
// const float TIMES[] = {0, 3000, 3100, 3500}; // milliseconds
// const double PRESSURES[] = {0, 20, 0, 0}; // PSI

// Reverse sawtooth trajectory
// const float TIMES[] = {0, 100, 3100, 3500}; // milliseconds
// const double PRESSURES[] = {0, 20, 0, 0}; // PSI

// Sinusoidal trajectory
// const float TIMES[] = {0, 1111, 2222, 3333, 4444, 5555, 6666, 7777, 8888, 10000}; // milliseconds
// const double PRESSURES[] = {
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

// Burst ramp trajectory w/o interp helper
// const float TIMES[] = {0, 5000, 10000, 15000, 20000, 
//                         25000, 30000, 35000, 40000, 45000, 
//                         50000, 55000, 60000, 65000, 70000, 
//                         75000, 80000, 80100, 80500}; // milliseconds
// const double PRESSURES[] = {0, 5, 5, 10, 10, 
//                             15, 15, 20, 20, 25, 
//                             25, 30, 30, 35, 35, 
//                             40, 40, 0, 0}; // PSI

// Linear ramp trajectory
// const float TIMES[] = {0, 180000, 180100}; // milliseconds
// const double PRESSURES[] = {0, 20, 0}; // PSI

// DO NOT CHANGE
const int TRAJ_SIZE = sizeof(TIMES) / sizeof(TIMES[0]);
