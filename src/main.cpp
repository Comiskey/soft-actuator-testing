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
unsigned long deltaT = 0;
bool cycleComplete = false;
double desiredPressure; // PSI
double output;

// Initialize controller and trajectory objects
AutoPID valvePID(&SensorPressure, &desiredPressure, &output, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
Trajectory traj(TRAJ_SIZE);

// helper function to display test end condition, # of cycles and exit
void endTest(String reason, int cycles){
  closeValves();
  valvePID.stop();
  setLCD(reason, "Cycles: " + String(totalCycles));
  exit(0);
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
  if(!InitializeTrajectory(&traj, TIMES, PRESSURES, TRAJ_SIZE)){
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
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);

  // vent any air in the system
  closeValves();
  delay(20); // 20 millisecond delay between signal and mechanical valve response
  openVentValve();
  delay(5000); // hold for 5 seconds to fully vent
  setLCD(F("Venting..."), F("Please wait"));
  closeValves(); 

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
  // Ensure that pressure does not exceed maximum
  if (PRESSURE_MAX < SensorPressure){
    endTest(F("Overpressure"), totalCycles);
  }
  // Check that controller isn't failing to follow the trajectory
  if(traj.failingToFollow(SensorPressure, deltaT, THRESHOLD)){
    endTest(F("Traj Follow Fail"), totalCycles);
  }
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
    logData(SensorPressure, valvePID.getPreviousError(), valvePID.getIntegral());
    lastPressureUpdate = millis();
  }
  // interpolate setpoint at 1/INTERP_CALC_DELAY Hz
  if ((millis() - lastInterpUpdate) > INTERP_CALC_DELAY){
    // calculate time since start of trajectory cycle
    deltaT = millis() - trajStartTime;
    // set pressure to interpolated value if deltaT
    // still lies within the trajectory's timeframe
    if (deltaT < TIMES[TRAJ_SIZE - 1]){
      desiredPressure = traj.interp(deltaT);
    }
    // otherwise, flag cycle as complete
    else{
      desiredPressure = PRESSURES[TRAJ_SIZE - 1];
      cycleComplete = true;
    }
    lastInterpUpdate = millis();
  }
  // control action
  valvePID.run();
  sendSignalToValves(output);
}



