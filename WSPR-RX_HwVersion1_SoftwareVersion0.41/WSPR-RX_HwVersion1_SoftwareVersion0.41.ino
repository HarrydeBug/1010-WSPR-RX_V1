
#include <EEPROM.h>
#include <si5351.h>
#include "Wire.h"
Si5351  si5351(0x62); //Normally address 0x60 but can by 0x62 in some cases, depends on what Si5351 version etc.


/* Software for Zachtek "HF WSPR DC RX"
   Uses EtherKit library for Si5351
   The Version of this software is stored in the constant "softwareversion" and is displayed on the Serialport att startup
   For Arduino Pro Mini with ATMega328

   0.2 Working Amplitude Trim.
   0.21 Seting LO to 10MHz When Trim is commanded, Amplitude trim disabled
   0.4 Triming of Crystal and Bandpass implemented + Fetch and Store Crystal Freq from EEPROM
*/


#define FreqJumper1 4        //Jumper 1, to set frequency
#define FreqJumper2 5        //Jumper 2
#define FreqJumper3 6        //Jumper 3
#define FreqJumper4 7        //Jumper 4
#define PCBButton_Enter 10   //Button used at Calibration
#define PCBButton_Minus 9    //Button used at Calibration
#define PCBButton_Plus 8     //Button used at Calibration
#define PCBTrimSwitch 11     //Switch "Trim/Operate"
#define PCBTrimIndicator 12  //LED used by Calibration routine
#define AudioADC A0          //Audio Out in to ADC 
#define LDO_Enable A2        //GPS Voltage regulator Enable on pin A2
#define VCCInADC  A3         //VCCIn to ADC (Not used by Software, for possible future firmware)

#define RXFREQ23cm          25930000000ULL   //23cm 1296.500,000MHz (5:th Overtone, not implemented)
#define RXFREQ70cm          14410000000ULL   //70cm  432.300,000MHz (3rd Overtone, not implemented)
#define RXFREQ2m            14448900000ULL   //2m    144.489,000MHz 
#define RXFREQ4m             7009100000ULL   //4m     70.091,000MHz 
#define RXFREQ6m             5029450000ULL   //6m     50.294,500MHz 
#define RXFREQ10m            2812460000ULL   //10m    28.124,600MHz
#define RXFREQ12m            2492460000ULL   //12m    24.924,600MHz 
#define RXFREQ15m            2109460000ULL   //15m    21.094.600MHz  
#define RXFREQ17m            1810460000ULL   //17m    18.104,600MHz
#define RXFREQ20m            1409560000ULL   //20m    14.095,600MHz 
#define RXFREQ30m            1013870000ULL   //30m    10.138,700MHz  
#define RXFREQ40m             703860000ULL   //40m     7.038,600MHz 
#define RXFREQ80m             356860000ULL   //80m     3.568,600MHz
#define RXFREQ160m            183660000ULL   //160m    1.836,600MHz
#define RXFREQ630m             47420000ULL   //630m      474.200kHz
#define RXFREQ2190m            13600000ULL   //2190m     136.000kHz

#define Crystal_load SI5351_CRYSTAL_LOAD_8PF  //8pF will give give good start Calibration on Hardware V1R4

const char softwareversion[] = "0.41" ; //Version of this program, sent to serialport at startup
uint32_t RefFreq = 25003000;   //Crystal frequency of the Si5351
uint64_t MyFrequency0 = 0 ;  //Wanted output freqency of the Si5351 in Hertz
  //Trim output freqency that will be +1500Hz of output 0 and only turned on when trimming


void setup() {
  pinMode(FreqJumper1, INPUT_PULLUP);
  pinMode(FreqJumper2, INPUT_PULLUP);
  pinMode(FreqJumper3, INPUT_PULLUP);
  pinMode(FreqJumper4, INPUT_PULLUP);
  pinMode(PCBButton_Enter, INPUT_PULLUP);
  pinMode(PCBButton_Minus, INPUT_PULLUP);
  pinMode(PCBButton_Plus, INPUT_PULLUP);
  pinMode(PCBTrimSwitch, INPUT_PULLUP);
  pinMode(PCBTrimIndicator, OUTPUT);
  pinMode(AudioADC, INPUT);
  pinMode(13, INPUT); //Fix V1R2 Hardware error by making both Pin 13 and A0 inputs
  pinMode(VCCInADC, INPUT);

  Serial.begin (57600);
  delay(500);//Wait for Serialport to be initialized properly
  Serial.print(F("Zachtek HF WSPR DC RX, Software version: "));
  Serial.println(softwareversion);

  Serial.println(F("Initializing.."));
  pinMode(LDO_Enable, OUTPUT); // Set Voltage Regulator Enable pin as output.
  Serial.println (F("Turning on 3.0V Voltage Regulator"));
  digitalWrite(LDO_Enable, HIGH); //Turn on 3.0V Power supply for the Si5351
  delay(10);//Give time for Si5351 to startup

  if (LoadFromEPROM()==false) //Read the crystal frequency from EEPROM
  {
    Serial.println ("No calibration data for the Crystal was found in EEPROM, using 25MHz");
    RefFreq = 25000000;
  }
  
  si5351.init(Crystal_load, RefFreq, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.output_enable(SI5351_CLK2, 0);
  SetStartFreq(); //Set LO Freq
  if (digitalRead(PCBTrimSwitch) == LOW)
  {
    Trim();
  }
}

void loop()
{
  /*
  if (digitalRead(PCBTrimSwitch) == LOW)
  {
    Trim();
  }
  if (digitalRead(PCBButton_Enter ) == LOW)
  {
    SetStartFreq();
  }
  delay (500);
  */
}




void Trim()
{
  int LEDState=0;
  int ADCVal, MinADC, MaxADC;
  Serial.println (F("Entering Trim mode"));
  digitalWrite(PCBTrimIndicator, HIGH); //Turn on TrimmerLED
  si5351.set_freq(1000000000UL, SI5351_CLK0);  //Set LO to 10MHz out for Calibration
  Serial.println (F("LO set to 10MHz, Measure at TestPoint 2 and use + and - buttons to change PLL Crystal Frequency in 100Hz steps"));
  Serial.println (F("When done press Enter to do fine adjustment"));
  //Keep doing until Enter Button is pressed
   while (digitalRead(PCBButton_Enter) == HIGH)
  {
    if (digitalRead(PCBButton_Plus) == LOW)
    {
      RefFreq=RefFreq+100;
      si5351.init(Crystal_load, RefFreq, 0);
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
      si5351.output_enable(SI5351_CLK0, 1);
      si5351.set_freq(1000000000UL, SI5351_CLK0);
      Serial.print (F("Crystal Freq set to:"));
      Serial.println (RefFreq);
      delay(200); 
    }

     if (digitalRead(PCBButton_Minus) == LOW)
    {
      RefFreq=RefFreq-100;
      si5351.init(Crystal_load, RefFreq, 0);
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
      si5351.output_enable(SI5351_CLK0, 1);
      si5351.set_freq(1000000000UL, SI5351_CLK0);
      Serial.print (F("Crystal Freq set to:"));
      Serial.println (RefFreq);
      delay(200); 
    }
  }
  digitalWrite(PCBTrimIndicator, LOW); //Turn off TrimmerLED
  delay(300);
  //Wait for Enter buton be released
   while (digitalRead(PCBButton_Enter) == LOW)
  {
  }
  digitalWrite(PCBTrimIndicator, HIGH); //Turn on TrimmerLED
  Serial.println (F("Fine adjust, use + and - buttons to adjust Frequency in 1Hz steps"));
  Serial.println (F("When done press Enter to save and go to bandpass trim routine"));
     while (digitalRead(PCBButton_Enter) == HIGH)
  {
    if (digitalRead(PCBButton_Plus) == LOW)
    {
      RefFreq=RefFreq+1;
      si5351.init(Crystal_load, RefFreq, 0);
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
      si5351.output_enable(SI5351_CLK0, 1);
      si5351.set_freq(1000000000UL, SI5351_CLK0);
      Serial.print (F("Crystal Freq set to:"));
      Serial.println (RefFreq);
      delay(200); 
    }

     if (digitalRead(PCBButton_Minus) == LOW)
    {
      RefFreq=RefFreq-1;
      si5351.init(Crystal_load, RefFreq, 0);
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
      si5351.output_enable(SI5351_CLK0, 1);
      si5351.set_freq(1000000000UL, SI5351_CLK0);
      Serial.print (F("Crystal Freq set to:"));
      Serial.println (RefFreq);
      delay(200); 
    }
  }
  digitalWrite(PCBTrimIndicator, LOW); //Turn off TrimmerLED
  delay(300);
  //Wait for Enter buton be released
   while (digitalRead(PCBButton_Enter) == LOW)
  {
  }
  digitalWrite(PCBTrimIndicator, HIGH); //Turn on TrimmerLED
  SaveToEPROM ();
  Serial.println (F("Crystal Frequency Saved"));
  Serial.println ("");
  Serial.println (F("Entering Bandpass trim routine, a input signal is generated at the antenna input."));
  Serial.println (F("This will generate a tone whose amplitude is measured by onboard ADC, use it to trim the bandpass filters."));
  Serial.println (F("Adjust C1 and C8 for max value"));
  Serial.println (F("Exit trim routine by moving the Trim swith to 'Operate'"));
  si5351.set_freq(MyFrequency0, SI5351_CLK0); //Set LO back to normal Frequency
  //si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
  si5351.output_enable(SI5351_CLK2, 1);//Turn on Output 2  
  si5351.set_freq(MyFrequency0+150000UL, SI5351_CLK2); //Set Output 2 to a output freqency that is set to +1500Hz of output 0, this will case a 1500Hz tone that can be trimed for signal strength 
  while (digitalRead(PCBTrimSwitch) == LOW)
  {
    if (LEDState==true)
    {
      digitalWrite(PCBTrimIndicator, LOW); //Turn off TrimmerLED
      LEDState=false;
    }
    else
    {
      digitalWrite(PCBTrimIndicator, HIGH); //Turn off TrimmerLED
      LEDState=true;
    }
    MinADC = 512;
    MaxADC = 512;
    for (int i = 0; i <= 300; i++) {
      ADCVal = analogRead(AudioADC);
      //Serial.println (ADCVal);
      if (ADCVal < MinADC) MinADC = ADCVal;
      if (ADCVal > MaxADC) MaxADC = ADCVal;
    }
    Serial.println (MaxADC - MinADC);
  }
  
  digitalWrite(PCBTrimIndicator, LOW); //Turn off TrimmerLED 
  si5351.set_freq(MyFrequency0+100000000UL, SI5351_CLK2); //In case of leakage set to higher freq than LO freq to avoid birdies ewen though output is turned off
  si5351.output_enable(SI5351_CLK2, 0); //Turn off Output 1 
  Serial.println (F("Exiting Trim mode"));
}


void SetStartFreq()
{
  int FreqJumperVal = 0;

  if (digitalRead(FreqJumper1) == LOW) FreqJumperVal = FreqJumperVal + 1 ;
  if (digitalRead(FreqJumper2) == LOW) FreqJumperVal = FreqJumperVal + 2  ;
  if (digitalRead(FreqJumper3) == LOW) FreqJumperVal = FreqJumperVal + 4  ;
  if (digitalRead(FreqJumper4) == LOW) FreqJumperVal = FreqJumperVal + 8  ;

  Serial.print (F("Frequency Jumper is set to "));
  Serial.print (FreqJumperVal);
  Serial.print (F(" so setting receiver frequency to: "));

  switch (FreqJumperVal) {
    case 0:
      MyFrequency0 = RXFREQ2190m;
      Serial.println (F("136.000kHz"));
      break;
    case 1:
      MyFrequency0 = RXFREQ630m;
      Serial.println (F("474.200kH"));
      break;
    case 2:
      MyFrequency0 = RXFREQ160m;
      Serial.println (F("1.836,600MHz"));
      break;
    case 3:
      MyFrequency0 = RXFREQ80m;
      Serial.println (F("3.568,600MHz"));
      break;
    case 4:
      MyFrequency0 = RXFREQ40m;
      Serial.println (F("7.038,600 MHz"));
      break;
    case 5:
      MyFrequency0 = RXFREQ30m;
      Serial.println (F("10.138,700MHz"));
      break;
    case 6:
      MyFrequency0 = RXFREQ20m;
      Serial.println (F("14.095,600MHz "));
      break;
    case 7:
      MyFrequency0 = RXFREQ17m;
      Serial.println (F("18.104,600MHz"));
      break;
    case 8:
      MyFrequency0 = RXFREQ15m;
      Serial.println (F("21.094.600MHz  "));
      break;
    case 9:
      MyFrequency0 = RXFREQ12m;
      Serial.println (F("24.924,600MHz"));
      break;
    case 10:
      MyFrequency0 = RXFREQ10m;
      Serial.println (F("28.124,600MHz"));
      break;
    case 11:
      MyFrequency0 = RXFREQ6m;
      Serial.println (F("50.294,500MHz "));
      break;
    case 12:
      MyFrequency0 = RXFREQ4m;
      Serial.println (F("70.091,000MHz "));
      break;
    case 13:
      MyFrequency0 = RXFREQ2m;
      Serial.println (F("144.489,000MHz"));
      break;
    case 14:
      MyFrequency0 = RXFREQ70cm;
      Serial.println (F("432.300,000MHz (3rd Overtone, LO=144.100,000MHz"));
      break;
    case 15:
      MyFrequency0 = RXFREQ23cm;
      Serial.println (F("1296.500,000MHz (9:th Overtone, LO=259.300.000MHz"));
      break;
    default:
   
      break;
  }
 
  si5351.set_freq(MyFrequency0, SI5351_CLK0);
}


unsigned long GetEEPROM_CRC(void) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (int index = 0 ; index < sizeof(RefFreq) ; ++index) {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}

void SaveToEPROM ()
{
unsigned long CRCFromEEPROM;
  EEPROM.put(0, RefFreq);         //Save the crystal Freqency to EEPROM at adress0
  CRCFromEEPROM=GetEEPROM_CRC ();    //Calculate CRC on the saved data
  EEPROM.put(sizeof(RefFreq), CRCFromEEPROM); //Save the CRC after the data
}


bool LoadFromEPROM (void)
{
unsigned long CRCFromEEPROM,CalculatedCRC;

  EEPROM.get(0, RefFreq);                      //Load all the data from EEPROM
  EEPROM.get(sizeof(RefFreq), CRCFromEEPROM);  //Load the saved CRC
  CalculatedCRC=GetEEPROM_CRC();                  //Calculate the CRC of the saved data
  return (CRCFromEEPROM==CalculatedCRC);          //If  Stored and Calculated CRC are the same then return true
}

