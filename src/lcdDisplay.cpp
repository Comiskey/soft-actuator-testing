#include <Arduino.h>
#include "lcdDisplay.h"

// Define LCD pins
LiquidCrystal lcd(10, 9, 8, 7, 3, 2);

void setLCD(String text1, String value) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(text1);
  lcd.setCursor(0, 1);
  lcd.print(value);
}
