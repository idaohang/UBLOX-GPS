/*
Copyright (C)2011 Kostas Tamateas <nosebleedKT@gmail.com>
This program is distributed under the terms of the GNU
General Public License, version 2. You may use, modify,
and redistribute it under the terms of this license.
A copy should be included with this source.
*/

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

