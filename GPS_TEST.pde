/*

Copyright (C)2011 Kostas Tamateas <nosebleedKT@gmail.com>
This program is distributed under the terms of the GNU
General Public License, version 2. You may use, modify,
and redistribute it under the terms of this license.
A copy should be included with this source.

*/

#include "GPS.h"

HardwareSerial GPS_PORT=Serial1;

GPS gps = GPS();

void setup()
{
  gps.setup(9600, GPS_PORT)
}

void loop()
{      
    gps.readSentence(GPS_PORT);
    Serial.println(gps.sentence);         
}

