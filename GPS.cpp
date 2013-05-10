/*

Copyright (C)2011 Kostas Tamateas <nosebleedKT@gmail.com>
This program is distributed under the terms of the GNU
General Public License, version 2. You may use, modify,
and redistribute it under the terms of this license.
A copy should be included with this source.

*/

#include "GPS.h"

GPS::GPS()
{
}

boolean GPS::setup(unsigned long baud, HardwareSerial &serialport)
{
  setDefaults();
    
  serialport.begin(baud);  
  
  // Serial.print("Setting Airbone mode..");  
  if(sendUBX(airbone1G, sizeof(airbone1G), serialport)) healthFlag |= 0b10000000;
  
  // Serial.print("Disabling SBAS..");
  if(sendUBX(SBASOFF, sizeof(SBASOFF), serialport)) healthFlag |= 0b00000001;
  
  // Serial.print("Disabling GGA..");
  if(sendUBX(GGAOFF, sizeof(GGAOFF), serialport)) healthFlag |= 0b00000010; 
  
  // Serial.print("Disabling GLL..");
  if(sendUBX(GLLOFF, sizeof(GLLOFF), serialport)) healthFlag |= 0b00000100;   
  
  // Serial.print("Disabling GSA..");
  if(sendUBX(GSAOFF, sizeof(GSAOFF), serialport)) healthFlag |= 0b00001000;  
  
  // Serial.print("Disabling GSV..");
  if(sendUBX(GSVOFF, sizeof(GSVOFF), serialport)) healthFlag |= 0b00010000;    
  
  // Serial.print("Disabling RMC..");
  if(sendUBX(RMCOFF, sizeof(RMCOFF), serialport)) healthFlag |= 0b00100000;  
  
  // Serial.print("Disabling VTG..");
  if(sendUBX(VTGOFF, sizeof(VTGOFF), serialport)) healthFlag |= 0b01000000;
  
  delay(1000);
  serialport.flush();
  
  if(healthFlag==0xFF) 
  {
    Serial.println("GPS Ok");     
    return true;
  }
  else 
  {
    Serial.println("GPS Nok"); 
    return false;
  }
}

boolean GPS::sendUBX(const byte *payload, byte len, HardwareSerial &serialport) 
{
  for(byte i=0; i<len; i++) 
    serialport.print(payload[i], BYTE);
   
  return UBXAck(payload, serialport);
}

boolean GPS::UBXAck(const byte *payload, HardwareSerial &serialport)
{
  byte ackByteID = 0;
  byte ackPacket[10];
  unsigned long startTime = millis();
  
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;  
  ackPacket[1] = 0x62;	
  ackPacket[2] = 0x05;	
  ackPacket[3] = 0x01;	
  ackPacket[4] = 0x02;	
  ackPacket[5] = 0x00;
  ackPacket[6] = payload[2];	
  ackPacket[7] = payload[3];	
  ackPacket[8] = 0;		
  ackPacket[9] = 0;	
  
  // Calculate the checksums
  for (byte i=2; i<8; i++) 
  {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
  
  while (1) 
  {
    // Test for success
    if (ackByteID > 9) 
    {
      // All packets in order!
      //if(DEBUG_ENABLED) DEBUG.println("Ok");
      return true;
    }
 
    // Timeout if no valid response
    if (millis() - startTime > 2000) 
    { 
      //if(DEBUG_ENABLED) DEBUG.println("Nok");
      return false;
    }
 
    // Make sure data is available to read
    if (serialport.available()) 
    {
      byte b = serialport.read();
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) 
      { 
        ackByteID++;
      } 
      else 
      {
        ackByteID = 0;	// Reset and look again, invalid order
      }
    }
  }  	
}

void GPS::readSentence(HardwareSerial &serialport)
{
  index = 0;   
  parity = 0;
  checksum = 0;
  setDefaults();   
  
  serialport.println("$PUBX,00*33");

  while(serialport.available() && index<sizeof(sentence))
  {  
      ch = serialport.read(); 

      if(ch == '$')
      {
        index = 0;
        continue;
      }
      
      if(ch == '*') break;
      
      sentence[index++] = ch;
      parity ^= ch;
  }  
 
  sentence[index] = '\0';

  //Serial.println(sentence);
 
  while(serialport.available())
  {
      ch = serialport.read();
      
      if(ch == 13)
        continue;
      
      if(ch == 10)
        break;
      
      if(checksum==0)
        checksum = 16 * hex2Byte(ch);   
      else 
        checksum += hex2Byte(ch);
  }
         
  if(sentence[0] == 'P')  // Be sure the sentence we got is a PUBX.
  { 
    if(checksum == parity)
    {
      parseUBX0();
      if(strcmp(NavStat, "NF") != 0 ) 
      {
        makeGlat();
        makeGlon(); 
        return;      
      }
    }
  }
  
  setDefaults();
}

void GPS::parseUBX0()
{
 byte commas=0, j=0;
 for(byte i=0; i<sizeof(sentence); i++)
 {
   if(sentence[i] == ',')
   {
     switch(++commas)
     {
        case 1: // UBX ID
          break; 
        case 2: // Time
          j=0;
          while(sentence[i+1+j] != ',' && j<sizeof(utcTime)-1) utcTime[j++] = sentence[i+1+j]; 
          utcTime[j] = 0;
          //Serial.println(utcTime);
          break;    
        case 3: // Latitude
          j=0;
          while(sentence[i+1+j] != ',' && j<sizeof(Latitude)-1) Latitude[j++] = sentence[i+1+j]; 
          Latitude[j] = 0;
          //Serial.print(Latitude);
          break; 
        case 4: // Equator
          Equator = sentence[i+1];
          break;  
        case 5: // Longitude
          j=0;
          while(sentence[i+1+j] != ',' && j<sizeof(Longitude)-1) Longitude[j++] = sentence[i+1+j]; 
          Longitude[j] = 0;
          //Serial.print(Longitude);
          break; 
        case 6: // Meridian, fixed length 1
          Meridian = sentence[i+1];
          break;  
        case 7: // Altitude in meters 
          j=0;
          while(sentence[i+1+j] != ',' && j<sizeof(Altitude)-1) Altitude[j++] = sentence[i+1+j];
          Altitude[j] = 0;
          //Serial.print(Altitude);
          break;
        case 8: // NavStat
          j=0;
          while(sentence[i+1+j] != ',' && j<sizeof(NavStat)-1) NavStat[j++] = sentence[i+1+j];
          NavStat[j] = 0;
          //Serial.println(NavStat);
          break;        
        case 9:  // Hacc
          break;    
        case 10: // Vacc
          break;
        case 11: // Speed over ground in km/h
          j=0;
          while(sentence[i+1+j] != ',' && j<sizeof(SOG)-1) SOG[j++] = sentence[i+1+j];
          SOG[j] = 0;
          //Serial.print(SOG);
          break;  
        case 12: // Course over ground in degrees
          j=0;
          while(sentence[i+1+j] != ',' && j<sizeof(COG)-1) COG[j++] = sentence[i+1+j];
          COG[j] = 0;
          //Serial.print(COG);
          break;      
        case 13: // Vertical velocity
          j=0;
          while(sentence[i+1+j] != ',' && j<sizeof(Vvel)-1) Vvel[j++] = sentence[i+1+j];
          Vvel[j] = 0;
          Vvel[0] == '-'? ascdesc='A' : ascdesc='D';
          //Serial.print(Vvel);
          break;   
        case 14: // Age of most recent DGPS corrections, empty = none available
          break;
        case 15: // HDOP
          j=0;
          while(sentence[i+1+j] != ',' && j<sizeof(HDOP)-1) HDOP[j++] = sentence[i+1+j];
          HDOP[j] = 0;
          //Serial.print(HDOP);         
          break;     
        case 16: // VDOP
          j=0;
          while(sentence[i+1+j] != ',' && j<sizeof(VDOP)-1) VDOP[j++] = sentence[i+1+j];
          VDOP[j] = 0;
          //Serial.print(VDOP);          
          break;
        case 17: // TDOP
          j=0;
          while(sentence[i+1+j] != ',' && j<sizeof(TDOP)-1) TDOP[j++] = sentence[i+1+j];
          TDOP[j] = 0;
          //Serial.print(TDOP);           
          break;              
        case 18: // Number of GPS satellites used in the navigation solution
          j=0;
          while(sentence[i+1+j] != ',' && j<sizeof(SatNum)-1) SatNum[j++] = sentence[i+1+j];
          SatNum[j] = 0;
          //Serial.print(SatNum);
          break;             
        default:
          return;  
     }     
    }
  }  
}

void GPS::setDefaults()
{  
  utcTime[0] = '0';
  utcTime[1] = '0';
  utcTime[2] = '0';
  utcTime[3] = '0';
  utcTime[4] = '0';
  utcTime[5] = '0';
  utcTime[6] = '0';
  utcTime[7] = '0';
  utcTime[8] = '0';
  utcTime[9] = 0;    

  Latitude[0] = '0'; 
  Latitude[1] = '0'; 
  Latitude[2] = '0';
  Latitude[3] = '0';
  Latitude[4] = '0';
  Latitude[5] = '0'; 
  Latitude[6] = '0'; 
  Latitude[7] = '0';
  Latitude[8] = '0';
  Latitude[9] = '0';
  Latitude[10] = 0;
  
  Equator = 'X';  
  
  Longitude[0] = '0'; 
  Longitude[1] = '0'; 
  Longitude[2] = '0';
  Longitude[3] = '0';  
  Longitude[4] = '0'; 
  Longitude[5] = '0'; 
  Longitude[6] = '0'; 
  Longitude[7] = '0'; 
  Longitude[8] = '0';
  Longitude[9] = '0';  
  Longitude[10] = '0'; 
  Longitude[11] = 0; 

  Meridian = 'Y';

  Altitude[0] = '0';
  Altitude[1] = '0';
  Altitude[2] = '0';
  Altitude[3] = '0';
  Altitude[4] = '0';
  Altitude[5] = '0';
  Altitude[6] = '0';
  Altitude[7] = '0';
  Altitude[8] = '0';
  Altitude[9] = 0;
  
  NavStat[0] = 'N';
  NavStat[1] = 'F';
  NavStat[2] = 0;

  SOG[0] = '0';
  SOG[1] = '0';
  SOG[2] = '0';
  SOG[3] = '0';
  SOG[4] = '0';
  SOG[5] = '0';
  SOG[6] = '0';
  SOG[7] = 0;

  COG[0] = '0';
  COG[1] = '0';
  COG[2] = '0';
  COG[3] = '0';
  COG[4] = '0';
  COG[5] = '0';
  COG[6] = 0;
  
  Vvel[0] = '0';
  Vvel[1] = '0';
  Vvel[2] = '0';
  Vvel[3] = '0';
  Vvel[4] = '0';
  Vvel[5] = '0';
  Vvel[6] = '0';
  Vvel[7] = 0;  
 
  SatNum[0] = '0';
  SatNum[1] = '0';
  SatNum[2] = 0;
  
  gLat[0] = '0';
  gLat[1] = '0';
  gLat[2] = '.';
  gLat[3] = '0';
  gLat[4] = '0';
  gLat[5] = '0';
  gLat[6] = '0';
  gLat[7] = 0;
  
  gLon[0] = '0';
  gLon[1] = '0';
  gLon[2] = '.';
  gLon[3] = '0';
  gLon[4] = '0';
  gLon[5] = '0';
  gLon[6] = '0';
  gLon[7] = 0;
  
  ascdesc='F';
}

void GPS::makeGlat()
{  
  char lat_minutes[8] = { Latitude[2], Latitude[3], Latitude[5], Latitude[6], Latitude[7], Latitude[8], Latitude[9], 0 };
  char lat_minutes_str[8];

  ltoa((atol(lat_minutes) / 60), lat_minutes_str, 10);

  gLat[0] = Latitude[0];
  gLat[1] = Latitude[1];
  gLat[2] = '.';

  if(lat_minutes[0]=='0' && lat_minutes[1]=='0' && lat_minutes[2]=='0') 
  {
    //snprintf(gLat, 8, "%i.000%1li", lat_degrees, lat_minutes); 
    gLat[3] = '0';
    gLat[4] = '0';
    gLat[5] = '0';
    gLat[6] = lat_minutes_str[0];
    gLat[7] = 0;
  }   
  else if(lat_minutes[0]=='0' && lat_minutes[1]=='0') 
  {
    //snprintf(gLat, 8, "%i.00%2li", lat_degrees, lat_minutes); 
    gLat[3] = '0';
    gLat[4] = '0';
    gLat[5] = lat_minutes_str[0];
    gLat[6] = lat_minutes_str[1];
    gLat[7] = 0;
  }
  else if(lat_minutes[0]=='0') 
  {
    //snprintf(gLat, 8, "%i.0%3li", lat_degrees, lat_minutes);
    gLat[3] = '0';
    gLat[4] = lat_minutes_str[0];
    gLat[5] = lat_minutes_str[1];
    gLat[6] = lat_minutes_str[2];
    gLat[7] = 0;    
  }
  else
  {
    //snprintf(gLat, 8, "%i.%4li", lat_degrees, lat_minutes);    
    gLat[3] = lat_minutes_str[0];
    gLat[4] = lat_minutes_str[1];
    gLat[5] = lat_minutes_str[2];
    gLat[6] = lat_minutes_str[3];
    gLat[7] = 0;       
  } 
}

void GPS::makeGlon()
{
  char lon_minutes[9] = { Longitude[3], Longitude[4], Longitude[6], Longitude[7], Longitude[8], Longitude[9], Longitude[10], Longitude[11], 0 };
  char lon_minutes_str[9];
 
  ltoa((atol(lon_minutes) / 60), lon_minutes_str, 10);

  gLon[0] = Longitude[1];
  gLon[1] = Longitude[2];
  gLon[2] = '.';
  
  if(lon_minutes[0]=='0' && lon_minutes[1]=='0' && lon_minutes[2]=='0') 
  {
    //snprintf(gLon, 8, "%i.000%1li", lon_degrees, lon_minutes);
    gLon[3] = '0';
    gLon[4] = '0';
    gLon[5] = '0';
    gLon[6] = lon_minutes_str[0];
    gLon[7] = 0;  
  }
  else if(lon_minutes[0]=='0' && lon_minutes[1]=='0') 
  {
    //snprintf(gLon, 8, "%i.00%2li", lon_degrees, lon_minutes); 
    gLon[3] = '0';
    gLon[4] = '0';
    gLon[5] = lon_minutes_str[0];
    gLon[6] = lon_minutes_str[1];
    gLon[7] = 0;    
  }
  else if(lon_minutes[0]=='0') 
  {
    //snprintf(gLon, 8, "%i.0%3li", lon_degrees, lon_minutes);
    gLon[3] = '0';
    gLon[4] = lon_minutes_str[0];
    gLon[5] = lon_minutes_str[1];
    gLon[6] = lon_minutes_str[2];
    gLon[7] = 0;      
  }
  else
  {
    //snprintf(gLon, 8, "%i.%4li", lon_degrees, lon_minutes);  
    gLon[3] = lon_minutes_str[0];
    gLon[4] = lon_minutes_str[1];
    gLon[5] = lon_minutes_str[2];
    gLon[6] = lon_minutes_str[3];
    gLon[7] = 0;        
  }
}

byte GPS::hex2Byte(char val) 
{
  return val >= 'A' && val <= 'F' ? val - 'A' + 10 : val - '0';
}
