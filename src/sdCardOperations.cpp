#include <Arduino.h>
#include <SdFat.h>
#include <SPI.h>
#include <AutoPID.h>
#include "sdCardOperations.h"
#include "lcdDisplay.h"
#include "analogPressureSensor.h"


// SD card variables
SdFat SD;
SdFile dataFile;
const int SD_CS_PIN = 4;
char fileName[20];
unsigned long testStartTime;

// Function to initialize the SD card
bool initializeSDCard() {
  setLCD(F("SD init..."), F("Please wait"));

  // Initialize SD card pins
  // be sure to use the correct pinnouts for your board
  pinMode(12, INPUT); // MISO
  pinMode(11, OUTPUT); // MOSI
  pinMode(13, OUTPUT); // SCK
  pinMode(SD_CS_PIN, OUTPUT); // CS

  // Initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards. Use SPI_FULL_SPEED for better performance.
  // THIS COULD BE PART OF THE ISSUE?
  if (!SD.begin(SD_CS_PIN, SD_SCK_MHZ(50))) {
    setLCD(F("SD Init"), F("Failed"));
    return false;
  }
  setLCD(F("Init Success"), F("SD Card Ready"));

  // Create a new file for each new cycle test
  int fileIndex = getNextFileIndex();
  snprintf(fileName, sizeof(fileName), "cycle_test_%d.csv", fileIndex);
  Serial.print(F("Creating file: "));
  Serial.println(fileName);
  if (!createFile(fileName)) {
    setLCD(F("File create fail"), F("Check SD Card"));
    return false;
  }

  // Open the file for writing
  if (!openFile(fileName)) {
    setLCD(F("File open fail"), F("Check SD Card"));
    return false;
  }

  Serial.println(F("SD card setup complete."));
  delay(2000); // delay for readability
  return true;
}

// helper function to create a new file
bool createFile(const char* fileName) {
  if (dataFile.open(fileName, O_RDWR | O_CREAT | O_AT_END)) {
    dataFile.println(F("time,pressure,error"));
    dataFile.close();
    return true;
  } else {
    setLCD(F("File create fail"), F("Check SD Card"));
    return false;
  }
}

// helper function to open an existing file
bool openFile(const char* fileName) {
  if (dataFile.open(fileName, O_RDWR | O_AT_END)) {
    return true;
  } else {
    setLCD(F("File open fail"), F("Check SD Card"));
    return false;
  }
}

// helper function to close the file
void closeSD() {
  if (dataFile) {
    dataFile.close();
  }
}

// helper function to get the next file index
// ensures data is not written over existing files
// when starting new cycle tests
int getNextFileIndex() {
  int index = 0;
  while (true) {
    snprintf(fileName, sizeof(fileName), "cycle_test_%d.csv", index);
    if (!SD.exists(fileName)) {
      return index;
    }
    index++;
  }
}

// write time, pressure, and error data to the SD card
void logData(double pressure, double error) {
  dataFile.print(millis() - testStartTime);
  dataFile.print(',');
  dataFile.print(pressure, 2);
  dataFile.print(',');
  dataFile.println(error, 2);
  dataFile.sync(); // Ensure data is written to the file
}

