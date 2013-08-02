// C5D v1.00a FIRMWARE
// ----------------------------------------------------------------------------
// Version:     1.1.0 -- Working.
// Date:        July 30, 2013
// Authors:     John and Nick @ JDS Labs, Inc.
// Requires:    Arduino Bootloader, Arduino 1.0.1
// License:     Creative Commons Attribution-ShareAlike 3.0 Unported
//              http://creativecommons.org/licenses/by-sa/3.0/deed.en_US

#include <Wire.h>
#include <EEPROM.h>

//LCD Stuff
#include <SoftwareSerial.h>
SoftwareSerial lcd(10, 1);


// PINOUTS (Note: Pinout of centerButton differs between C5 (centerButton=0) and C5D (centerButton=3)
int upButton = 2;       // Attenuation UP pushbutton
int downButton = 5;     // Attenuation DOWN pushbutton
int centerButton = 3;   // Center pushbutton
int PWRVOLLED = 4;      // PCB LED1 (bi-color); Green LED is wired active-low
int ENPOSREG = 6;       // Pin to enable +7V LDO
int ENNEGREG = 7;       // Pin to enable -7V LDO
int GAIN = A1;          // Pin to control gain
int POTPOWER = A2;      // 5V to DS1882 IC logic
int PREBOOST = A7;      // USB or battery voltage, 2.7-5.5VDC
int LLF = A0;           // Low latency filter of PCM5102 DAC. Set High by default
int DAC5VEN = 9;        // Controles Enable pin of 5V LDO supplying DAC circuitry. Activate before DACPWREN.
int DACPWREN = 8;       // Controls Enable pins of all 3.3V LDO's supplying DAC circuitry

// TEMPORARY VARIABLES
int uptemp = 0;
int downtemp = 0;
int centertemp = 0;                          // NOTE: Attenuation & bass should be stored in non-volatile memory later. Pots remember their value; MCU does not.
byte gainstate = 0;                          // Stores current gain, where 1 = low, 0 = high
byte lastStoredGainState = 0;                // Last stored EEPROM gain
byte attenuation = 62;                       // Attenuation value of volume potentiometer gangs. Initially set to minimum volume.
byte PriorAtten = 62;                        // Hols attenuation during mute function
byte tempattenuation = 0;                    // Holds attenuation value for second pot bitwise operation
byte lastStoredAttenuation = 0;              // Last stored EEPROM volume
float BattVoltage = 5;                       // Temp variable to read battery voltage
int flashState = LOW;                        // Flashing power LED during low battery
int DACPowerEnable = LOW;                    // Temp state of DAC power; off by default
unsigned long lastMillis = 0;                // Time of last battery check
unsigned long volumeChangeTimer = 0;         // Time of Last Volume Change
unsigned long gainChangeTimer = 0;           // Time of Last Gain Change
unsigned long startTime = 0;                 // Start time for Volume Change Debounce
boolean lowBatt = false;                     // Low Battery Indicator
float lastKnowBattVoltage = 0;               // Battery Voltage for iPad Operations....poorly commented 
float VoltageUponIpad = 0;                   // Recorded Battery voltage at time of iPad connection
boolean isIpad = false;                      // If the amp thinks it is an iPad...

// CONFIGURATION CONSTANTS
int DACFilterState = HIGH;                   // Default state of PCM5102A's low latency filter: High = enabled, Low = disabled
int VolDelay = 375;                          // Delay (milliseconds) before volume control quickly transitions through steps
int StepPause = 55;                          // Delay (milliseconds) between each volume step in seek mode. Minimum value is 55ms, due to write time of DS1882
int PwrCheck = 500;                          // Delay (milliseconds) between periodically checking battery volume. Directly affects speed of low battery LED flashing
float LowVoltageThreshold = 3.51;            // 'Low' hysteresis threshold (voltage at which low battery flashing begins). 3.50V should yield 45-90 minute warning.
float HighVoltageThreshold = 3.55;           // 'High' hysteresis threshold for low battery detection

void setup()
{
  delay(50);                                 // Wait for power to stabilize, then setup I/O pins
  pinMode(LLF, OUTPUT);
  pinMode(DAC5VEN, OUTPUT);
  pinMode(DACPWREN, OUTPUT);
  
  pinMode(POTPOWER, OUTPUT);
  pinMode(ENNEGREG, OUTPUT);
  pinMode(ENPOSREG, OUTPUT);
  pinMode(GAIN, OUTPUT);
  pinMode(PWRVOLLED, OUTPUT);
  pinMode(upButton, INPUT);                      // Declare pushbuttons as inputs
  pinMode(downButton, INPUT);
  pinMode(centerButton, INPUT);

  digitalWrite(ENPOSREG, HIGH);                  // Enable +7V LDO
  digitalWrite(ENNEGREG, HIGH);                  // Enable -7V LDO
  delay(200);                                    // Wait for power to stabilize
  digitalWrite(PWRVOLLED, LOW);                  // Turn Power LED on, turn gain and bass LEDs off
  digitalWrite(POTPOWER, HIGH);                  // Enable DS1882 IC
  delay(50); 
  digitalWrite(GAIN, LOW);                       // Set gain HIGH
  
  // Setup DAC (OFF by default) 
  DACPowerEnable = LOW;                           
  digitalWrite(DAC5VEN, LOW); 
  delay(25);                                    // Wait for 5V power to stabilize before enabling 3.3V regulators
  digitalWrite(DACPWREN, LOW); 
  
  
  // Setup Serial
  Wire.begin();                                  // Join the I2C bus as master device
  
  // Setup LCD
  delay(3000);
  lcd.begin(9600);                              // start lcd serial
 
 
  // Sets DS1882 registers to nonvolatile memory, zero-crossing enabled, and 64 tap positions (pg 10 of datasheet)
  // FUTURE: This only needs to be performed once--read register and only write if necessary!
  tempattenuation = B10000110;
  Wire.beginTransmission(40);                    // Set parameters of volume potentiometers
  Wire.write(tempattenuation);
  Wire.endTransmission();
  delay(25);
  
  // Retrieve volume and gain values from EEPROM. Set default values if EEPROM is new or corrupted.
  attenuation = EEPROM.read(0);
  delay(4);
  gainstate = EEPROM.read(1);
  
  if(gainstate > 1)
  {
    gainstate = 1;
  }
  changeGain();

  if(attenuation > 63)
  {
    attenuation = 62;
  }
  changeVolume();

}


void loop()
{
  uptemp = digitalRead(upButton);                  // Read the up pushbutton
  downtemp = digitalRead(downButton);              // Read the down pushbutton
  centertemp = digitalRead(centerButton);          // Read the center pushbutton


 // User pressed center pushbutton
  if (centertemp == HIGH) {
    delay(50);                                     // Simple debounce: wait and see if user actually pushed the button
    centertemp = digitalRead(centerButton);        // Read the center pushbutton again
    if (centertemp == HIGH) {                      // If user pressed center pushbutton, toggle gain
        if(gainstate < 1){
          gainstate = 1;
        }
        else{
          gainstate = 0;
        }
        
        changeGain();
        changeLEDs();
        delay(333);                                // Wait for 1/3 of a second to prevent additional button presses
    }
  }
  
    // --------------Volume Control---------------
    // First IF statement LOWERS volume. DS1808 has a range of 0-33; DS1882 has range of 0-63. So, for DS1882:
    //       attenuation = 0 = max volume
    //       attenuation = 62 = minimum volume
    //       attenuation = 63 = mute
    if (uptemp == HIGH || downtemp == HIGH) {
      delay(StepPause);                                // Simple debounce: wait and see if user actually pushed the button
      uptemp = digitalRead(upButton);                  // Update the up pushbutton
      downtemp = digitalRead(downButton);              // Update the down pushbutton
                                                        
      if ((uptemp == HIGH) && (attenuation < 63)) {   // Check if button was really pressed and dealing with the direction  
          attenuation++;                              // If user pressed button and volume isn't already at min.
          
          clearDisplay(); 
          lcd.println(attenuation);
      }

      if ((downtemp == HIGH) && (attenuation > 0)) {  // If user pressed button and volume isn't already at max.
          attenuation--;                              // Decrease the potentiometer attenuation value
          
          clearDisplay(); 
          lcd.println(attenuation);
      }
      
      changeVolume();                                 // Perform initial volume change
      startTime = millis();                           // Record starting time of volume seek
     
     while((millis() - startTime) < VolDelay){        // Check to see if user changed button presses
          uptemp = digitalRead(upButton);             // Update the up pushbutton
          downtemp = digitalRead(downButton);         // Update the down pushbutton
          
          // Exit loop if user releases button
          if ((uptemp != HIGH) && (downtemp != HIGH)){
            break;
          }
       }
      
      if ((downtemp == HIGH) && (attenuation > 0)) {   // If button was not released, go Up fast
          do{
            downtemp = digitalRead(downButton);        // Update the down pushbutton
            delay(StepPause);                          // Delay between each step
            attenuation--;                             // Increment attenuation
          
            clearDisplay(); 
          lcd.println(attenuation);
          
            changeVolume();                            // Perform volume change
          }while((downtemp == HIGH) && (attenuation > 0));    // Ensure attenuation is within valid range
      }
      
      if ((uptemp == HIGH) && (attenuation < 63)) {    // If button was not released, go Down fast
          do{
            uptemp = digitalRead(upButton);           // Update the up pushbutton
            delay(StepPause);                         // Delay between each step
            attenuation++;                            // Decrement attenuation
            
            clearDisplay(); 
          lcd.println(attenuation);
          
            changeVolume();                           // Perform volume change
          }while((uptemp == HIGH) && (attenuation < 63));    // Ensure attenuation is within valid range
      }
    }
    // -------------End Volume Control-------------
    
  if ((millis() - lastMillis) > PwrCheck)             // Check battery voltage
  {      checkBattery();
        lastMillis = millis();
  }
  
  if ((millis() - volumeChangeTimer > 2000) || (millis() - gainChangeTimer > 2000)){
         if (lastStoredAttenuation != attenuation){
             EEPROM.write(0, attenuation);            // Store attenuation value in EEPROM location 0, wait 6ms for write to complete
             delay(6);
             lastStoredAttenuation = attenuation;
             volumeChangeTimer = millis();
         }
         else if (lastStoredGainState != gainstate){    
             EEPROM.write(1, gainstate);              // Store gain value in EEPROM location 1        
             delay(6);  
             lastStoredGainState = gainstate;
             gainChangeTimer = millis();  
              
       }
     }
}

void changeVolume()
{
      // Update first potentiometer gang, where pot address is first two bits of attenuation: 00 ######
      Wire.beginTransmission(40);                 // Transmit to DS1808, device #40 (0x28) = 0101 000, LSB is auto sent by Arduino Wire
      Wire.write(attenuation);                    // Write wiper value
      Wire.endTransmission();                     // Stop transmitting    
      
      // Update second potentiometer gang. Same process, where pot address is now: 01 ######.
      // (each I2C transmission should be started stopped individually)
      tempattenuation = B01000000 | attenuation;          // Attenuation value is bitwise OR'ed with address byte fpr second pot gang
      Wire.beginTransmission(40);
      Wire.write(tempattenuation);
      Wire.endTransmission();
      volumeChangeTimer = millis();       
}

void changeGain()
{
  digitalWrite(GAIN, gainstate);
  gainChangeTimer = millis();  

}

void clearDisplay()
{
  lcd.write(0xFE);  // send the special command
  lcd.write(0x01);  // send the clear screen command
}

void changeLEDs()
{
    if(flashState == LOW && lowBatt == true){       // If the battery is low, toggle the LED according to flashState
      digitalWrite(PWRVOLLED, flashState); 
    }
    
    else if(lowBatt == true && flashState == HIGH){
        digitalWrite(PWRVOLLED, flashState); 
    }
    
    else{                                         // Activate LED as usual if the battery isn't low
        digitalWrite(PWRVOLLED, LOW); 
    } 
}

void turnDacOn(){                                   // Mutes amp, turns DAC on and waits 2sec for DAC to prepare, then un-mutes
   mute();              
   DACPowerEnable = HIGH;                           
   digitalWrite(DAC5VEN, HIGH); 
   delay(25);                                       // Wait for 5V power to stabilize before enabling 3.3V regulators
   digitalWrite(DACPWREN, HIGH); 
   digitalWrite(LLF, DACFilterState);               // Set low latency filter to default value 
   delay(2000);
   UnMute();
}

void turnDacOff(){                                 // Mutes amp, turns DAC off and waits 2sec for DAC to prepare, then un-mutes
   mute();
   digitalWrite(LLF, LOW);                         // Set FLT/LLF low (otherwise 5V is applied to an unpowered chip) 
   DACPowerEnable = LOW;                           
   digitalWrite(DACPWREN, LOW); 
   delay(25); 
   digitalWrite(DAC5VEN, LOW);                     // Wait for 5V power to stabilize before enabling 3.3V regulators 
   delay(2000);
   UnMute();
}

void mute()                                       // Records current volume, then sets amp to mute
{
    PriorAtten = attenuation;
    attenuation = 63;
    changeVolume();
}

void UnMute(){                                   // Restores volume to level prior to muting
    attenuation = PriorAtten;
    changeVolume();
}

// checkBattery monitors operating voltage and sets DAC power and LED flashing accordingly
void checkBattery()                                       
{
  lastKnowBattVoltage = 0;
    lastKnowBattVoltage = BattVoltage;
    BattVoltage = (float)analogRead(PREBOOST)*4.95/1023;   // Note: 4.95V is imperical value of C5's 5V rail
    
    lcd.println(lastKnowBattVoltage, 2);
    
    if(BattVoltage > HighVoltageThreshold){                // Use of hysteresis to avoid erratic LED toggling
      flashState = LOW;                                    // flashState toggles if voltage is below the hysteresis threshold,
      lowBatt = false;                                     // lowBatt prevents flashState from toggling when above high threshold
    }
    else if(BattVoltage < LowVoltageThreshold) {          // lowbatt toggles 
      lowBatt = true;
    }
  
    if(lowBatt == true && flashState == LOW){            // If the battery is low, determine next state of LED
      flashState = HIGH;
    }
    else{
      flashState = LOW;
    }
    
    // Logic below enables/disables DAC. DAC must not be turnd off after connection to an iPad (iPad voltage unpredictable).
    // If voltage was previously between 4.10-4.95V, but has dropped to battery voltage in < 500ms, an iPad might be connected.
    if (lastKnowBattVoltage  >= 4.1 && lastKnowBattVoltage < 4.95 && BattVoltage < 4.1){
     isIpad = true;
     VoltageUponIpad = BattVoltage;
     turnDacOn(); 
    }
    
    // If voltage exceeds iPad's capability, it must not be an iPad...
    if ((lastKnowBattVoltage > 4.9) && (BattVoltage > 4.80) && isIpad == true && (BattVoltage > VoltageUponIpad + 0.12)){
      isIpad = false;                        
    }
    
    // Conditions for "PC Mode"
    if(BattVoltage >= 4.1 && isIpad == false && DACPowerEnable == LOW){        // If DAC is off and USB cable is connected, enable DAC                              
      turnDacOn();
      lcd.println("PC - DAC ON");
      lcd.println(BattVoltage, 2);
      delay(1000);
      
    }
    else if (DACPowerEnable == HIGH && BattVoltage < 4.1 && isIpad == false){     // If DAC is on and USB cable is unplugged, disable DAC
        turnDacOff();   
        lcd.println("PC - DAC OFF");
        lcd.println(BattVoltage, 2);
        delay(1000);        
        }
        
    changeLEDs();                                        // Call LED function to perform toggle
  }
