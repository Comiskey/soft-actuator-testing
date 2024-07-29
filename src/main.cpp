// Library Inclusions
#include <Arduino.h>
#include <Wire.h>
#include <AutoPID.h>

// Custom Inclusions
#include "adjustableSettings.h"
#include "sdCardOperations.h"
#include "analogPressureSensor.h"
#include "valveControl.h"
#include "lcdDisplay.h"
#include "trajectory.h"

// Declare dynamic variables
float lastPressureUpdate; // milliseconds
float lastInterpUpdate; // milliseconds
float cycleStartTime; // milliseconds
float trajStartTime; // milliseconds
int totalCycles = 0;
unsigned long deltaT;
bool cycleComplete = true;
double desiredPressure; // PSI
double output;

// Initialize controller and trajectory objects
AutoPID valvePID(&SensorPressure, &desiredPressure, &output, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
Trajectory traj(TRAJ_SIZE);

// helper function to display test end condition, # of cycles and exit
void endTest(String reason, int cycles){
  closePressureValve();
  vent();
  valvePID.stop();
  setLCD(reason, "Cycles: " + String(totalCycles));
  exit(0);
}

void setup() {
  Serial.begin(115200);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.clear();

  // Initialize valves and pressure sensor
  pinMode(PRESSURE_PIN, OUTPUT);
  pinMode(VENT_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT);
  analogReference(DEFAULT);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);

  // vent any air in the system
  closeValves();
  delay(20); // 20 millisecond delay between signal and mechanical valve response
  openVentValve();
  setLCD(F("Venting..."), F("Please wait"));
  delay(5000); // hold for 5 seconds to fully vent
  closeValves(); 

  // Initialize SD card
  if (!initializeSDCard()) {
    while (1);
  }

  // Initialize trajectory
  if(!InitializeTrajectory(&traj, TIMES, PRESSURES, TRAJ_SIZE)){
    setLCD(F("Traj Error"), F("Check Serial"));
    while(1);
  }

   // Set PID Controller settings
  valvePID.setTimeStep(CONTROLLER_DELAY); //milliseconds.
  // Tells AutoPID to not use bang-bang control
  valvePID.setBangBang(0, 0);

  // hold until start button pressed.
  while(digitalRead(START_BUTTON_PIN) == HIGH){
    setLCD(F("Press Start"), F("to Begin Test"));
  }
  delay(2000);
  
  // begin cycle test
  testStartTime = millis();
  traj.reset();
}

void loop() {
  // Stop test if stop button is pressed
  if (digitalRead(STOP_BUTTON_PIN) == LOW){
    endTest(F("Stop Pressed"), totalCycles);
  }
  // Check that pressure does not exceed maximum
  if (PRESSURE_MAX < SensorPressure){
    vent();
    endTest(F("Overpressure"), totalCycles);
  }
  // Check that controller isn't failing to follow the trajectory
  if(traj.failingToFollow(SensorPressure, deltaT, THRESHOLD)){
    endTest(F("Traj Follow Fail"), totalCycles);
  }
  // Check if cycle is complete
  if (cycleComplete){
    totalCycles++;
    setLCD(String(fileName), "Cycles: " + String(totalCycles));
    valvePID.reset(); // anti-windup call
    traj.reset();
    logData(SensorPressure, valvePID.getPreviousError(), valvePID.getIntegral(), cycleComplete);
    trajStartTime = millis();
    cycleComplete = false;
  }
  // Log data at 1/PRESSURE_READ_DELAY Hz
  if ((millis() - lastPressureUpdate) > PRESSURE_READ_DELAY){
    UpdateFilteredSensorPressure();
    logData(SensorPressure, valvePID.getPreviousError(), valvePID.getIntegral(), cycleComplete);
    // reset pressure update timer
    lastPressureUpdate = millis();
  }
  // interpolate setpoint at 1/INTERP_CALC_DELAY Hz
  if ((millis() - lastInterpUpdate) > INTERP_CALC_DELAY){
    // calculate time since start of trajectory cycle
    deltaT = millis() - trajStartTime;
    // check if trajectory is finished
    if (traj.isFinished(deltaT)){
      
      desiredPressure = PRESSURES[TRAJ_SIZE - 1];
      cycleComplete = true;
    }
    else{ // if not, interpolate current setpoint
      desiredPressure = traj.interp(deltaT);
    }
    // reset interpolation timer
    lastInterpUpdate = millis();
  }
  // control action
  valvePID.run();
  sendSignalToValves(output);
}



