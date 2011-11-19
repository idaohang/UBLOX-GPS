/*
  Chipset UBX-G5010
  
  UART (TTL) interface
  Digital I/O Voltage Level 1.65 – 3.6 V

  Output Message Format
  GPS Protocol: NMEA, UBX binary
  GGA, GLL, GSA, GSV, RMC, VTG, TXT 

  Operational limits 
  Velocity: 500 m/s
  Altitude: 50,000 m
  
  Environmental Characteristics
  Operating Temperature - 40°C to + 85°C
  
  Power
  Power supply voltage 3.6v
  Peak Supply Current Max=150mA
  Max Performance Mode Acquisition = 74mA
  Max Performance Mode Tracking = 47mA
  Eco Mode Acquisition = 67mA
  Eco Mode Tracking = 43mA
*/

#ifndef __GPS_H__
#define __GPS_H__

#include <WProgram.h>
#include <EEPROM.h>
#include "Settings.h"

const byte grDatum[] = { 0xB5, 0x62, 0x06, 0x06, 0x02, 0x00, 0x48, 0x00, 0x56, 0xDA };

const byte airbone1G[] = { 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
                           0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
                           
const byte SBASOFF[] = { 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x00, 0x07, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x87, 0x29 };
const byte GGAOFF[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24 };
const byte GLLOFF[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B };
const byte GSAOFF[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32 };
const byte GSVOFF[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39 };
const byte RMCOFF[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40 };
const byte VTGOFF[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47 };

const byte startGPS[] = { 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x09, 0x00, 0x17, 0x76 };
const byte stopGPS[] = { 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x08, 0x00, 0x16, 0x74 };

const byte maxMode[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x00, 0x00, 0x19, 0x81 };
const byte ecoMode[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x00, 0x04, 0x1D, 0x85 };

class GPS
{
  public:
    GPS();
    boolean setup();
    void start();
    void stop();
    void restart();
    boolean sendUBX(const byte *, byte);
    boolean UBXAck(const byte *);
    void readSentence();
    char utcTime[10];    // UTC Time, fixed length 9
    char Latitude[11];   // Latitude, fixed length 10
    char Equator;        // Equator, fixed length 1
    char Longitude[12];  // Longitude, fixed length 11
    char Meridian;       // Meridian, fixed length 1
    char Altitude[10];   // Altitude above user datum ellipsoid, meters, max length 9
    char NavStat[3];     // Navigation Status, fixed length 2
    char SOG[8];         // Speed over ground, km/h, max length 7
    char COG[7];         // Course over ground, degrees, fixed length 6
    char Vvel[9];        // Vertical velocity, positive=downwards
    char HDOP[6];        // Horizontal Dilution of Precision
    char VDOP[6];        // Vertical Dilution of Precision
    char TDOP[6];        // Time Dilution of Precision
    char SatNum[3];      // Number of GPS satellites used in the navigation solution, max length 2
    char gLat[8];        // Google Lat
    char gLon[8];        // Google Lon
    char sentence[115];
    
  private:
    byte hex2Byte(char);
    void parseUBX0();
    void calcCoords();
    void recAltitude();
    void setDefaults(boolean);

    char ch; 
    byte index;   
    byte parity;
    byte checksum;
    byte healthFlag;
    unsigned int highestAlt;
};

#endif
