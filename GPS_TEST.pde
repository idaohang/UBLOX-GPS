#include "GPS.h"

GPS gps = GPS();

void setup()
{
  gps.setup();
}

void loop()
{      
    gps.readSentence();
    Serial.println(gps.sentence);         
}

