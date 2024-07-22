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
unsigned int holdStartTime;
unsigned long cycleStartTime;
int totalCycles = 0;
const int holdTime = 3000; // milliseconds
const bool step = false; // for debugging w/ step function. To be removed in final version

// PID Controller variables
double desiredPressure = 25.0; // Example setpoint in PSI for step function
double deadband = 1; // PSI
double output = 0.0;
int outputMin = -1.0;
int outputMax = 1.0;
double Kp = 0.12; // Clark's defaults: Kp = 0.1, Ki = 0.001, Kd = 0.0. They do not currenlty work with this set-up
double Ki = 0.001;
double Kd = 0.0;
// See autoPID library for more information
AutoPID valvePID(&SensorPressure, &desiredPressure, &output, outputMin, outputMax, Kp, Ki, Kd);

// Trajectory variables
// WARNING: Ensure the first and last pressures of your trajectory path
// match so as to ensure smooth transitions between cycles
// Example step function trajectory
const float times[] = {100, 2000, 2100}; //milliseconds
const double pressures[] = {20, 20, 0}; // PSI

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
Trajectory traj(sizeof(times) / sizeof(times[0])); // Do not change

// closed-loop PID actuation control function
// See helper function definitons in helper files
void actuate(){
  valvePID.reset();
  output = 0; // TOCHECK
  while (!valvePID.atSetPoint(deadband)) {
      // statements for debugging
      Serial.print("Current Pressure: ");
      Serial.print(SensorPressure);
      Serial.print(" PSI, PID Output: ");
      Serial.println(output);
      // control logic
      valvePID.run();
      sendSignalToValves(output);
      getFilteredSensorPressure();
      // Sd card logging
      logPressureData();
      // anti-integral windup
      if (valvePID.getIntegral()>1.0){
        valvePID.setIntegral(0);
      }
    }
  closeValves();
}

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
  if(!InitializeTrajectory(&traj, times, pressures, sizeof(times) / sizeof(times[0]))){
    setLCD("Traj Error", "Check Serial");
    while(1);
  }

  // Initialize valves and pressure sensor
  pinMode(PRESSURE_PIN, OUTPUT);
  pinMode(VENT_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT);
  analogReference(DEFAULT);

  // Set PID Controller settings
  valvePID.setTimeStep(100); //milliseconds. Adjust to 50
  // Tells AutoPID to not use bang-bang control
  valvePID.setBangBang(0, 0);

  // vent any air in the system
  vent();
  delay(2000); // hold for 2 seconds to fully vent
  closeValves(); 

  delay(2000);
  setLCD("Ready", "");
  delay(2000);
  setLCD("Setpoint: ", String(desiredPressure));

  // begin test cycle
  testStartTime = millis();
}

void loop() {
  // update and log sensor pressure variables
  getFilteredSensorPressure();
  logPressureData();
  if (step) { // PID tuning loop
    // debugging statements. To be removed
    Serial.println("Starting actuation to setpoint...");
    Serial.println(desiredPressure);
    Serial.println(SensorPressure);
    // Actuate to setpoint
    actuate();
    // hold at setpoint for holdTime
    Serial.println("Reached setpoint. Holding pressure...");
    holdStartTime = millis();
    while (millis() - holdStartTime < holdTime) {
      getFilteredSensorPressure();
      logPressureData();
      Serial.print("Current Pressure: ");
      Serial.println(SensorPressure);
    }
    // deflate to ~0 PSI
    Serial.println("Deflating to ~0 PSI...");
    vent();
    while (SensorPressure > 0.5) {
      // log deflation action
      getFilteredSensorPressure();
      logPressureData();
      // debugging statements. To be removed
      Serial.print("Current Pressure: ");
      Serial.println(SensorPressure);
    }
    closeValves();
    // hold at 0 PSI for holdTime to complete
    // step function cycle
    holdStartTime = millis();
    while (millis() - holdStartTime < holdTime) {
      // log deflation action
      getFilteredSensorPressure();
      logPressureData();
      // debugging statements. To be removed
      Serial.print("Current Pressure: ");
      Serial.println(SensorPressure);
    }
    totalCycles++;
    // debugging statements. To be removed
    Serial.print("Cycle completed. Total Cycles: ");
    Serial.println(totalCycles);
  } 
  else { // otherwise, use trajectory based loop
    // interpolation variables
    unsigned long deltaT = 0;
    unsigned long trajStartTime = millis();
    delay(200); // initial 0.2 sec delay to interpolate a nonzero value
    // Interpolate over trajectory
    while (1) {
      deltaT = millis() - trajStartTime;
      if (deltaT >= traj.times[traj.maxSize - 1]){
        break;
      }
      desiredPressure = traj.interp(deltaT);
      // debugging statements. To be removed
      Serial.print("DeltaT: ");
      Serial.println(deltaT);
      Serial.print("Starting actuation to setpoint: ");
      Serial.println(desiredPressure);
      Serial.println(SensorPressure);
      // Actuate to interpolated setpoint
      actuate();
    }

    // Once trajectory has been completely followed, 
    // actuate to last point for smooth transition
    // to start of next trajectory cycle
    desiredPressure = pressures[traj.maxSize - 1];
    actuate();
    totalCycles++;
    Serial.print("Cycle completed. Total Cycles: ");
    Serial.println(totalCycles);
    traj.reset();
  }
}
