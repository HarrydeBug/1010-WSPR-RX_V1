#include <EEPROM.h>
#include <si5351.h>


/* Software for Zachtek "HF WSPR DC RX"
   The Version of this software is stored in the constant "softwareversion" and is displayed on the Serialport att startup
   For Arduino Pro Mini ATMega 328 and ATMega168

   0.2 Working Amplitude Trim.
   0.21 Seting LO to 10MHz When Trim is commanded, Amplitude trim disabled
   0.4 Triming of Crystal and Bandpass implemented + Fetch and Store Crystal Freq from EEPROM
   0.42 Changes from EtherKit to Direct acces for Si5351 to save memory
   0.43 Corrected 6m frequency
*/

#define I2C_START 0x08
#define I2C_START_RPT 0x10
#define I2C_SLA_W_ACK 0x18
#define I2C_SLA_R_ACK 0x40
#define I2C_DATA_ACK 0x28
#define I2C_WRITE 0b11000000
#define I2C_READ 0b11000001
#define SI5351A_H

#define SI_CLK0_CONTROL 16 // Register definitions
#define SI_CLK1_CONTROL 17
#define SI_CLK2_CONTROL 18
#define SI_SYNTH_PLL_A 26
#define SI_SYNTH_PLL_B 34
#define SI_SYNTH_MS_0 42
#define SI_SYNTH_MS_1 50
#define SI_SYNTH_MS_2 58
#define SI_PLL_RESET 177

#define SI_R_DIV_1 0b00000000 // R-division ratio definitions
#define SI_R_DIV_2 0b00010000
#define SI_R_DIV_4 0b00100000
#define SI_R_DIV_8 0b00110000
#define SI_R_DIV_16 0b01000000
#define SI_R_DIV_32 0b01010000
#define SI_R_DIV_64 0b01100000
#define SI_R_DIV_128 0b01110000

#define SI_CLK_SRC_PLL_A 0b00000000
#define SI_CLK_SRC_PLL_B 0b00100000

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
#define RXFREQ6m             5029300000ULL   //6m     50.294,500MHz 
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


const char softwareversion[] = "0.43" ; //Version of this program, sent to serialport at startup
uint32_t RefFreq ;                      //Crystal frequency of the Si5351
uint64_t MyFrequency0 = 0 ;             //Wanted output freqency of the Si5351 in Hertz
  


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
    Serial.print ("No calibration data for the Crystal was found in EEPROM, using ");
    RefFreq = 26000000;
    Serial.print  (RefFreq); 
    Serial.println (" Hz");
  }
  
  i2cInit();
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
  si5351aSetFrequency(1000000000UL);  //Set LO to 10MHz out for Calibration
  Serial.println (F("LO set to 10MHz, Measure at TestPoint 2 and use + and - buttons to change PLL Crystal Frequency in 100Hz steps"));
  Serial.println (F("When done press Enter to do fine adjustment"));
  //Keep doing until Enter Button is pressed
   while (digitalRead(PCBButton_Enter) == HIGH)
  {
    if (digitalRead(PCBButton_Plus) == LOW)
    {
      RefFreq=RefFreq+100;
     
      si5351aSetFrequency(1000000000UL);
      Serial.print (F("Crystal Freq set to:"));
      Serial.println (RefFreq);
      delay(200); 
    }

     if (digitalRead(PCBButton_Minus) == LOW)
    {
      RefFreq=RefFreq-100;
    
      si5351aSetFrequency(1000000000UL);
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
     
      si5351aSetFrequency(1000000000UL);
      Serial.print (F("Crystal Freq set to:"));
      Serial.println (RefFreq);
      delay(200); 
    }

     if (digitalRead(PCBButton_Minus) == LOW)
    {
      RefFreq=RefFreq-1;
      si5351aSetFrequency(1000000000UL);
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
 
  Serial.println (F("Exit trim routine by moving the Trim swith to 'Operate'"));
  si5351aSetFrequency(MyFrequency0); //Set LO back to normal Frequency
   
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
      Serial.println (F("14.095,600MHz"));
      break;
    case 7:
      MyFrequency0 = RXFREQ17m;
      Serial.println (F("18.104,600MHz"));
      break;
    case 8:
      MyFrequency0 = RXFREQ15m;
      Serial.println (F("21.094.600MHz"));
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
      Serial.println (F("50.293,000MHz"));
      break;
    case 12:
      MyFrequency0 = RXFREQ4m;
      Serial.println (F("70.091,000MHz"));
      break;
    case 13:
      MyFrequency0 = RXFREQ2m;
      Serial.println (F("144.489,000MHz"));
      break;
    case 14:
      MyFrequency0 = RXFREQ70cm;
      Serial.println (F("432.300,000MHz (3rd Overtone, LO=144.100,000MHz)"));
      break;
    case 15:
      MyFrequency0 = RXFREQ23cm;
      Serial.println (F("1296.500,000MHz (9:th Overtone, LO=259.300.000MHz)"));
      break;
    default:
   
      break;
  }

  si5351aSetFrequency(MyFrequency0);
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

  

uint8_t i2cStart()
{
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

  while (!(TWCR & (1 << TWINT))) ;

  return (TWSR & 0xF8);
}

void i2cStop()
{
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

  while ((TWCR & (1 << TWSTO))) ;
}

uint8_t i2cByteSend(uint8_t data)
{
  TWDR = data;

  TWCR = (1 << TWINT) | (1 << TWEN);

  while (!(TWCR & (1 << TWINT))) ;

  return (TWSR & 0xF8);
}

uint8_t i2cByteRead()
{
  TWCR = (1 << TWINT) | (1 << TWEN);

  while (!(TWCR & (1 << TWINT))) ;

  return (TWDR);
}

uint8_t i2cSendRegister(uint8_t reg, uint8_t data)
{
  uint8_t stts;

  stts = i2cStart();
  if (stts != I2C_START) return 1;

  stts = i2cByteSend(I2C_WRITE);
  if (stts != I2C_SLA_W_ACK) return 2;

  stts = i2cByteSend(reg);
  if (stts != I2C_DATA_ACK) return 3;

  stts = i2cByteSend(data);
  if (stts != I2C_DATA_ACK) return 4;

  i2cStop();

  return 0;
}

uint8_t i2cReadRegister(uint8_t reg, uint8_t *data)
{
  uint8_t stts;

  stts = i2cStart();
  if (stts != I2C_START) return 1;

  stts = i2cByteSend(I2C_WRITE);
  if (stts != I2C_SLA_W_ACK) return 2;

  stts = i2cByteSend(reg);
  if (stts != I2C_DATA_ACK) return 3;

  stts = i2cStart();
  if (stts != I2C_START_RPT) return 4;

  stts = i2cByteSend(I2C_READ);
  if (stts != I2C_SLA_R_ACK) return 5;

  *data = i2cByteRead();

  i2cStop();

  return 0;
}

// Init TWI (I2C)
//
void i2cInit()
{
  TWBR = 92;
  TWSR = 0;
  TWDR = 0xFF;
  PRR = 0;
}

//
// Set up specified PLL with mult, num and denom
// mult is 15..90
// num is 0..1,048,575 (0xFFFFF)
// denom is 0..1,048,575 (0xFFFFF)
//
void setupPLL(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom)
{
  uint32_t P1; // PLL config register P1
  uint32_t P2; // PLL config register P2
  uint32_t P3; // PLL config register P3

  P1 = (uint32_t)(128 * ((float)num / (float)denom));
  P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
  P2 = (uint32_t)(128 * ((float)num / (float)denom));
  P2 = (uint32_t)(128 * num - denom * P2);
  P3 = denom;

  i2cSendRegister(pll + 0, (P3 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 1, (P3 & 0x000000FF));
  i2cSendRegister(pll + 2, (P1 & 0x00030000) >> 16);
  i2cSendRegister(pll + 3, (P1 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 4, (P1 & 0x000000FF));
  i2cSendRegister(pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 &
                  0x000F0000) >> 16));
  i2cSendRegister(pll + 6, (P2 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 7, (P2 & 0x000000FF));
}

//
// Set up MultiSynth with integer Divider and R Divider
// R Divider is the bit value which is OR'ed onto the appropriate
// register, it is a #define in si5351a.h
//
void setupMultisynth(uint8_t synth, uint32_t Divider, uint8_t rDiv)
{
  uint32_t P1; // Synth config register P1
  uint32_t P2; // Synth config register P2
  uint32_t P3; // Synth config register P3

  P1 = 128 * Divider - 512;
  P2 = 0; // P2 = 0, P3 = 1 forces an integer value for the Divider
  P3 = 1;

  i2cSendRegister(synth + 0, (P3 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 1, (P3 & 0x000000FF));
  i2cSendRegister(synth + 2, ((P1 & 0x00030000) >> 16) | rDiv);
  i2cSendRegister(synth + 3, (P1 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 4, (P1 & 0x000000FF));
  i2cSendRegister(synth + 5, ((P3 & 0x000F0000) >> 12) | ((P2 &
                  0x000F0000) >> 16));
  i2cSendRegister(synth + 6, (P2 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 7, (P2 & 0x000000FF));
}

//
// Switches off Si5351a output
// Example: si5351aOutputOff(SI_CLK0_CONTROL);
// will switch off output CLK0
//
void si5351aOutputOff(uint8_t clk)
{
  i2cSendRegister(clk, 0x80); // Refer to SiLabs AN619 to see
  //bit values - 0x80 turns off the output stage
}




//
// Set CLK0 output ON and to the specified frequency
// Frequency is in the range 10kHz to 150MHz and given in centiHertz (hundreds of Hertz)
// Example: si5351aSetFrequency(1000000200);
// will set output CLK0 to 10.000,002MHz
//
// This example sets up PLL A
// and MultiSynth 0
// and produces the output on CLK0
void si5351aSetFrequency(uint64_t frequency) //Frequency is in centiHz
{
  static uint64_t oldFreq;
  int32_t FreqChange;
  uint64_t pllFreq;
  uint32_t l;
  float f;
  uint8_t mult;
  uint32_t num;
  uint32_t denom;
  uint32_t Divider;
  uint8_t rDiv;

  // Serial.print (F("Freq is="));
  // Serial.println (uint64ToStr(frequency,false));
  if (frequency > 100000000) { //If higher than 1MHz then set output divider to 1
    rDiv = SI_R_DIV_1;
    Divider = 90000000000ULL / frequency;// Calculate the division ratio. 900MHz is the maximum internal (expressed as deciHz)
    pllFreq = Divider * frequency; // Calculate the pllFrequency:
    mult = pllFreq / (RefFreq * 100UL); // Determine the multiplier to
    l = pllFreq % (RefFreq * 100UL); // It has three parts:
    f = l; // mult is an integer that must be in the range 15..90
    f *= 1048575; // num and denom are the fractional parts, the numerator and denominator
    f /= RefFreq; // each is 20 bits (range 0..1048575)
    num = f; // the actual multiplier is mult + num / denom
    denom = 1048575; // For simplicity we set the denominator to the maximum 1048575
    num = num / 100;
  }
  else // lower freq than 1MHz - set output Divider to 128
  {
    rDiv = SI_R_DIV_128;
    frequency = frequency * 128ULL; //Set base freq 128 times higher as we are dividing with 128 in the last output stage
    Divider = 90000000000ULL / frequency;// Calculate the division ratio. 900,000,000 is the maximum internal
    pllFreq = Divider * frequency; // Calculate the pllFrequency:
    mult = pllFreq / (RefFreq * 100UL); // Determine the multiplier to
    l = pllFreq % (RefFreq * 100UL); // It has three parts:
    f = l; // mult is an integer that must be in the range 15..90
    f *= 1048575; // num and denom are the fractional parts, the numerator and denominator
    f /= RefFreq; // each is 20 bits (range 0..1048575)
    num = f; // the actual multiplier is mult + num / denom
    denom = 1048575; // For simplicity we set the denominator to the maximum 1048575
    num = num / 100;
  }


  // Set up PLL A with the calculated  multiplication ratio
  setupPLL(SI_SYNTH_PLL_A, mult, num, denom);

  // Set up MultiSynth Divider 0, with the calculated Divider.
  // The final R division stage can divide by a power of two, from 1..128.
  // reprented by constants SI_R_DIV1 to SI_R_DIV128 (see si5351a.h header file)
  // If you want to output frequencies below 1MHz, you have to use the
  // final R division stage
  setupMultisynth(SI_SYNTH_MS_0, Divider, rDiv);

  // Reset the PLL. This causes a glitch in the output. For small changes to
  // the parameters, you don't need to reset the PLL, and there is no glitch
  FreqChange = frequency - oldFreq;

  if ( abs(FreqChange) > 100000) //If changed more than 1kHz then reset PLL (completely arbitrary choosen)
  {
    i2cSendRegister(SI_PLL_RESET, 0xA0);
  }

  // Finally switch on the CLK0 output (0x4F)
  // and set the MultiSynth0 input to be PLL A
  i2cSendRegister(SI_CLK0_CONTROL, 0x4F | SI_CLK_SRC_PLL_A);
  oldFreq = frequency;
}
