//-------------------------------------------------------------------
// Sous Vide Controller
// Bill Earl - for Adafruit Industries
//
// Based on the Arduino PID and PID AutoTune Libraries 
// by Brett Beauregard
//
// Modified for thermistor, LCD and one button setup
// by Nicolas Aimon
//------------------------------------------------------------------

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// So we can save and retrieve settings
#include <EEPROM.h>

// Library for the basic liquid crystal
#include <LiquidCrystal.h>

// ************************************************
// Pin definitions
// ************************************************

// Output Relay
#define RelayPin 6

// Input Thermistor
#define ThermPin A1

// Button
#define PushPin 10

// Potentiometer
#define PotPin A2

// Thermistor variable
#define NUMSAMPLES 5

// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;

volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;

double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);

// ************************************************
// DiSplay Variables and constants
// ************************************************

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SETP, RUN, TUNE_P, TUNE_I, TUNE_D, AUTO};
operatingState opState = OFF;

// Button handling
int buttonState;
int lastButtonState = LOW;
long lastButtonTime = 0;
long longPressTime = 500;
boolean shortButtonPressed = false;
boolean longButtonPressed = false;

boolean change = false;
long timeout = 10000;

// thermistor setup
int nominalR = 10000;
int nominalT = 25;
int numsamp = NUMSAMPLES;
int bcoeff = 3950;
int seriesR = 10000;
int samples[NUMSAMPLES];

LiquidCrystal lcd(12, 11, 1, 2, 3, 4);

// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup()
{
   // Initialize Relay Control:
   pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
   digitalWrite(RelayPin, LOW);  // make sure it is off to start

   // Initialize LCD DiSplay 

   lcd.begin(16, 2);
   lcd.print(F("    Naimo"));
   lcd.setCursor(0, 1);
   lcd.print(F("   Sous Vide!"));
   delay(3000);
  
   // Initialize the PID and related variables
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);

  // Run timer2 interrupt every 15 ms 
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect) 
{
  if (opState == OFF)
  {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
   lcd.clear();
   resetbutton();
  
   switch (opState)
   {
   case OFF:
      Off();
      break;
   case SETP:
      Tune_Sp();
      break;
    case RUN:
      Run();
      break;
   case TUNE_P:
      TuneP();
      break;
   case TUNE_I:
      TuneI();
      break;
   case TUNE_D:
      TuneD();
      break;
   }
}

// read temperature
double measuretemp(){
  uint8_t i;
  float average;
  for (i=0; i< numsamp; i++) {
    samples[i] = analogRead(ThermPin);
    delay(10);
  }
  average = 0;
  for (i=0; i< numsamp; i++) {
    average += samples[i];
  }
  average /= numsamp;
  average = 1023 / average - 1;
  average = seriesR / average;
  float steinhart;
  steinhart = average / nominalR;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= bcoeff;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (nominalT + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  return steinhart;
}

// capture button presses
void button(){
  buttonState = digitalRead(PushPin);
  if (buttonState != lastButtonState) {
    if (buttonState == LOW) {
      if (millis()-lastButtonTime < longPressTime){
        shortButtonPressed = true;
      }
      else{
        longButtonPressed = true;
      }
    } 
    lastButtonState = buttonState;
    lastButtonTime = millis();
  }
}

void resetbutton(){
  shortButtonPressed=false;
  longButtonPressed=false;
}

// ************************************************
// Initial State - long or short button to enter setpoint
// ************************************************
void Off()
{
   myPID.SetMode(MANUAL);
   digitalWrite(RelayPin, LOW);  // make sure it is off
   lcd.setCursor(0, 0);
   lcd.print(F("   Sous Vide"));
   lcd.setCursor(0, 1);
   lcd.print(F("   OFF"));
   
   while(!(shortButtonPressed || longButtonPressed))
   {
      button();
   }
   
   //turn the PID on
   myPID.SetMode(AUTOMATIC);
   windowStartTime = millis();
   opState = RUN; // start control
}

// ************************************************
// Setpoint Entry State
// Potentiometer to change setpoint
// Short for tuning parameters
// Long for changing value
// ************************************************
void Tune_Sp()
{
   lcd.print(F("Set Temperature:"));
   while(true)
   {
      button();
      if (change){
        Setpoint = map(analogRead(PotPin), 0, 1023, 99, 15);
        lcd.setCursor(15, 1);
        lcd.print("#");
      };        
      if (longButtonPressed)
      {
         change =!change;
         return;
      }
      if (shortButtonPressed)
      {
         opState = TUNE_P;
         change=false;
         return;
      }

      if ((millis() - lastButtonTime) > timeout)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         change=false;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Setpoint);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Proportional Tuning State
// Potentiometer to change Kp
// Short for Ki
// Long for changing value
// ************************************************
void TuneP()
{
   lcd.print(F("Set Kp"));
   while(true)
   {
      button();
      if (change){
        Kp = map(analogRead(PotPin), 0, 1023, 1000, 500);
        lcd.setCursor(15, 1);
        lcd.print("#");
      };        
      if (longButtonPressed)
      {
         change =!change;
         return;
      }
      if (shortButtonPressed)
      {
         opState = TUNE_I;
         change=false;
         return;
      }
      if ((millis() - lastButtonTime) > timeout)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         change=false;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kp);
      lcd.print(" ");
      DoControl();
   }
}


// ************************************************
// Integral Tuning State
// Potentiometer to change Ki
// Short for Kd
// Long for changing value
// ************************************************
void TuneI()
{
   lcd.print(F("Set Ki"));
   while(true)
   {
      button();

      if (change){
        Ki = map(analogRead(PotPin), 0, 1023, 1000, 100)/1000.0;
        lcd.setCursor(15, 1);
        lcd.print("#");
      };        
      if (longButtonPressed)
      {
         change =!change;
         return;
      }
      if (shortButtonPressed)
      {
         opState = TUNE_D;
         change=false;
         return;
      }
      if ((millis() - lastButtonTime) > timeout)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         change=false;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Ki);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Derivative Tuning State
// Potentiometer to change Kd
// Short for RUN
// Long for changing value
// ************************************************
void TuneD()
{
   lcd.print(F("Set Kd"));
   while(true)
   {
      button();

      if (change){
        Kd = map(analogRead(PotPin), 0, 1023, 200, 50)/1000.0;
        lcd.setCursor(15, 1);
        lcd.print("#");
      };        
      if (longButtonPressed)
      {
         change =!change;
         return;
      }
      if (shortButtonPressed)
      {
         opState = RUN;
         change=false;
         return;
      }
      if ((millis() - lastButtonTime) > timeout)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         change=false;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kd);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// PID Control State
// Long for autotune
// Short - off
// ************************************************
void Run()
{
   // set up the LCD's number of rows and columns: 
   lcd.print(F("Sp: "));
   lcd.print(Setpoint);
   lcd.print(F("C : "));

   SaveParameters();
   myPID.SetTunings(Kp,Ki,Kd);
   while(true)
   {
      button();
      if (shortButtonPressed)
      {
//         StartAutoTune();
           opState = SETP;
           return;
      }
      else if (longButtonPressed && (abs(Input - Setpoint) < 0.5))  // Should be at steady-state
      {
         StartAutoTune();
         resetbutton();
      }
    
      DoControl();
      
      lcd.setCursor(0,1);
      lcd.print(Input);
      lcd.print(F("C : "));
      
      float pct = map(Output, 0, WindowSize, 0, 1000);
      lcd.setCursor(10,1);
      lcd.print(F("      "));
      lcd.setCursor(10,1);
      lcd.print(pct/10);
      lcd.print("%");

      lcd.setCursor(15,0);
      if (tuning)
      {
        lcd.print("T");
      }
      else
      {
        lcd.print(" ");
      }
      
      delay(100);
   }
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  Input = measuretemp();

  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
        FinishAutoTune();
     }
  }
  else // Execute control algorithm
  {
     myPID.Compute();
  }
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output; 
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime)))
  {
     digitalWrite(RelayPin,HIGH);
  }
  else
  {
     digitalWrite(RelayPin,LOW);
  }
}

// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 60;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }  
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}
