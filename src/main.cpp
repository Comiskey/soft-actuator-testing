// Library Inclusions
#include <Arduino.h>
#include <Wire.h>
#include <AutoPID.h>

// Custom Inclusions
#include "sdCardOperations.h"
#include "analogPressureSensor.h"
#include "valveControl.h"
#include "lcdDisplay.h"
#include "trajectory.h"

// Variable Declarations
unsigned int lastPressureUpdate;
unsigned int lastInterpUpdate;
unsigned long cycleStartTime;
unsigned long trajStartTime;
int totalCycles = 0;
unsigned long deltaT = 0;
bool cycleComplete = false;
const int PRESSURE_READ_DELAY = 10; // milliseconds aka 100Hz
const int INTERP_CALC_DELAY = 20; // milliseconds aka 50Hz

// PID Controller variables
double desiredPressure; // PSI, set by code automatically based on trajectory
double deadband = 1; // PSI
double output = 0.0;
int outputMin = -1.0;
int outputMax = 1.0;
double Kp = 0.12; // Clark's defaults: Kp = 0.1, Ki = 0.001, Kd = 0.0. Good starting point
double Ki = 0.001;
double Kd = 0.0;
// See autoPID library for more information
AutoPID valvePID(&SensorPressure, &desiredPressure, &output, outputMin, outputMax, Kp, Ki, Kd);

// Trajectory variables
// WARNING: Ensure the first and last pressures of your trajectory path
// match so as to ensure smooth transitions between cycles
// Example step function trajectory
// const float times[] = {0, 100, 2000, 2100, 3000}; //milliseconds
// const double pressures[] = {0, 20, 20, 0, 0}; // PSI

// Example sinusoidal trajectory
const float times[] = {0, 1111, 2222, 3333, 4444, 5555, 6666, 7777, 8888, 10000}; // milliseconds
const double pressures[] = {
    0, 
    20 * sin(M_PI / 9 * 1), 
    20 * sin(M_PI / 9 * 2), 
    20 * sin(M_PI / 9 * 3), 
    20 * sin(M_PI / 9 * 4), 
    20 * sin(M_PI / 9 * 5), 
    20 * sin(M_PI / 9 * 6), 
    20 * sin(M_PI / 9 * 7), 
    20 * sin(M_PI / 9 * 8), 
    0 }; // PSI

const int trajSize = sizeof(times) / sizeof(times[0]);
Trajectory traj(trajSize); // Do not change

void setup() {
  Serial.begin(115200);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.clear();

  // Initialize SD card
  if (!initializeSDCard()) {
    while (1);
  }

  // Initialize trajectory
  if(!InitializeTrajectory(&traj, times, pressures, trajSize)){
    setLCD(F("Traj Error"), F("Check Serial"));
    while(1);
  }

  // update LCD with current status
  setLCD(F("Trajectory Set"), F("Venting..."));

  // Initialize valves and pressure sensor
  pinMode(PRESSURE_PIN, OUTPUT);
  pinMode(VENT_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT);
  analogReference(DEFAULT);

  // Set PID Controller settings
  valvePID.setTimeStep(50); //milliseconds. Adjust to 50 
  // Tells AutoPID to not use bang-bang control
  valvePID.setBangBang(0, 0);

  // vent any air in the system
  vent();
  delay(2000); // hold for 2 seconds to fully vent
  closeValves(); 
  delay(2000);

  // hold until start button pressed. TO BE IMPLEMENTED
  setLCD(F("Test Ready:"), F("Press Start"));
  delay(2000);
  
  // begin cycle test
  testStartTime = millis();
}

void loop() {
  // Check if previous cycle has completed
  if (cycleComplete){
    totalCycles++;
    setLCD(String(fileName), "Cycles: " + String(totalCycles));
    valvePID.reset(); // anti-windup call
    traj.reset();
    trajStartTime = millis();
    cycleComplete = false;
  }
  // Log data at 1/PRESSURE_READ_DELAY Hz
  if ((millis() - lastPressureUpdate) > PRESSURE_READ_DELAY){
    UpdateFilteredSensorPressure();
    logData(SensorPressure, valvePID.getPreviousError());
    lastPressureUpdate = millis();
  }
  // interpolate setpoint at 1/INTERP_CALC_DELAY Hz
  if ((millis() - lastInterpUpdate) > INTERP_CALC_DELAY){
    // calculate time since start of trajectory cycle
    deltaT = millis() - trajStartTime;
    // set pressure to interpolated value if deltaT
    // still lies within the trajectory's timeframe
    if (deltaT < times[trajSize - 1]){
      desiredPressure = traj.interp(deltaT);
    }
    // otherwise, flag cycle as complete
    else{
      desiredPressure = pressures[trajSize - 1];
      cycleComplete = true;
    }
    lastInterpUpdate = millis();
  }
  // control action
  valvePID.run();
  sendSignalToValves(output);
}
