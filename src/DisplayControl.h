#include "LiquidCrystal_I2C.h"
#include "Arduino.h"
#include "string.h"

#ifndef SPDCTRLDSPLY
#define SPDCTRLDSPLY

class DisplayControl // (0x27, 16, 2)
{
private:
  const unsigned long displayInterval = 200;
  unsigned long prevTime;
  LiquidCrystal_I2C *lcd;

public:
  DisplayControl(int displayAddress, int displayCols, int displayRows)
  {
    lcd = new LiquidCrystal_I2C(displayAddress, displayCols, displayRows);
  }
  ~DisplayControl()
  {
    lcd->clear();
    lcd->noBacklight(); 

    delete lcd;
  }

  void init()
  {
    lcd->init();
    lcd->backlight();
  }

  void displayLine(int row, String text, bool clear = false);

  static String appendTextWithSpace(String leftString, String rightString)
  {
    String dt = leftString;
    dt.concat(" ");
    dt.concat(rightString);
    return dt;
  }

   static String appendTextWithSpace(String rightString[], int length)
  {
    String dt = "";
    for (int i = 0; i < length; i++)
    {
      dt.concat(" ");
      dt.concat(rightString[i]);
    }
    
    return dt;
  }
};

#endif