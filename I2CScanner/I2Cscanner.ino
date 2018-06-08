/* Software for Zachtek "HF WSPR DC RX"
   The Version of this software is stored in the constant "softwareversion" and is displayed on the Serialport att startup
   For Arduino Pro Mini

   This software is only usefull for finding hardware errors of finding out what I2C adress the Si5351 is using.
   The WSPR receiver does not receive anthing while this software is loaded in to teh Arduino.
   
   This software scans the I2C hardware. If it finds something it displays the I2C address
   Orginal I2C code by Nick Gamon
   The rest of the code is written by Harry Zachrisson at ZachTek
   

// I2C Scanner
// Written by Nick Gammon
// Date: 20th April 2011M
*/




#define FreqJumper1 4    //Jumper 1, to set frequency
#define FreqJumper2 5    //Jumper 2
#define FreqJumper3 6    //Jumper 3
#define FreqJumper4 7    //Jumper 4
#define PCBButton_Enter 10   //Button used at Calibration
#define PCBButton_Minus 9    //Button used at Calibration
#define PCBButton_Plus 8     //Button used at Calibration
#define PCBTrimSwitch 11     //Switch "Trim/Operate"
#define PCBTrimIndicator 12  //LED used by Calibration routine
#define AudioADC 13          //Audio Out in to ADC 
#define LDO_Enable A2        //GPS Voltage regulator Enable on pin A2
#define VCCInADC  A3         //VCCIn to ADC (Not used by Software, for possible future firmware)

#include <Wire.h>

void setup() {
   pinMode(FreqJumper1, INPUT_PULLUP);  
  pinMode(FreqJumper2, INPUT_PULLUP);
  pinMode(FreqJumper3, INPUT_PULLUP);
  pinMode(FreqJumper4, INPUT_PULLUP);
  pinMode(PCBButton_Enter, INPUT_PULLUP);
  pinMode(PCBButton_Minus, INPUT_PULLUP);
  pinMode(PCBButton_Plus, INPUT_PULLUP);
  pinMode(PCBTrimSwitch, INPUT);
  pinMode(PCBTrimIndicator, OUTPUT);
  pinMode(AudioADC, INPUT);
  pinMode(VCCInADC, INPUT);
  
  Serial.begin (57600);
  delay(500);//Wait for Serialport to be initialized properly
  Serial.println(F("Zachtek HF WSPR DC RX I2C scanner, based on Nick Gammons work"));
  
  
  Serial.println(F("Initializing.."));
  pinMode(LDO_Enable, OUTPUT); // Set Voltage Regulator Enable pin as output.
  Serial.println (F("Turning on 3.0V Voltage Regulator"));
  digitalWrite(LDO_Enable, HIGH); //Turn on 3.0V Power supply for the Si5351

  Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  
  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
      {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
      } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
}  // end of setup

void loop() {}

