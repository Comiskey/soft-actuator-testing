#ifndef SD_CARD_OPERATIONS_H
#define SD_CARD_OPERATIONS_H

#include <SdFat.h>
#include <SPI.h>
#include <analogPressureSensor.h>

extern SdFat SD;
extern SdFile dataFile;
extern unsigned long testStartTime;
extern char fileName[20];

bool initializeSDCard();
void logData(double pressure, double error);
void closeSD();
bool createFile(const char* fileName);
bool openFile(const char* fileName);
int getNextFileIndex();

#endif
