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
#include <OneWire.h>
#include <DallasTemperature.h>

// So we can save and retrieve settings
#include <EEPROM.h>

// Library for the basic liquid crystal
#include <LiquidCrystal.h>

// ************************************************
// Pin definitions
// ************************************************

// Output Relay
#define RelayPin 9

// Button
#define PushPin 8

// Potentiometer
#define PotPin A0

#define ONE_WIRE_BUS 12

// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;
double Time;
double Minutes;
long StartTime;
boolean TimerOn = false;

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
const int TimeAddress = 32;

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
enum operatingState { OFF = 0, SETP, SETT, RUN, TUNE_P, TUNE_I, TUNE_D, AUTO, DONE};
operatingState opState = OFF;

// Button handling
int buttonState;
int lastButtonState = HIGH;
long lastButtonTime = 0;
long longPressTime = 500;
boolean shortButtonPressed = false;
boolean longButtonPressed = false;

boolean change = false;
long timeout = 10000;

LiquidCrystal lcd(11, 10, 5, 4, 3, 2);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// ************************************************

// Setup and diSplay initial screen
// ************************************************
void setup()
{
   //sensors.begin();
  // Initialize Relay Control:
   pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
   digitalWrite(RelayPin, LOW);  // make sure it is off to start

   pinMode(PushPin, INPUT_PULLUP);
   
   // Initialize LCD DiSplay 

   lcd.begin(16, 2);
   lcd.print(F("    Wuhbet"));
   lcd.setCursor(0, 1);
   lcd.print(F("   Sous Vide!"));
   delay(3000);
  
   // Initialize the PID and related variables
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);

  // Run Timer2 interrupt every 15 ms 
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
   case SETT:
      Tune_T();
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
   case AUTO:
   Auto();
      break;
   case DONE:
      Done();
      break;      
   }
}

// read temperature
double measuretemp(){
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

// capture button presses
void button(){
  buttonState = digitalRead(PushPin);
  if (buttonState != lastButtonState) {
    if (buttonState == HIGH) {
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
   lcd.setCursor(1, 0);
   lcd.print(F("Sous Vide Off"));
   
   while(!(shortButtonPressed || longButtonPressed))
   {
     Input = measuretemp();
     lcd.setCursor(4,1);
     lcd.print(int(Input/10) % 10);
     lcd.print(int(Input) % 10);
     lcd.print(F("."));
     lcd.print(int(Input*10) % 10);
     lcd.print(F(" C"));
     button();
   }
   
   //turn the PID on
   myPID.SetMode(AUTOMATIC);
   windowStartTime = millis();
   opState = RUN; // start control
}

void Done()
{
   myPID.SetMode(MANUAL);
   digitalWrite(RelayPin, LOW);  // make sure it is off
   lcd.setCursor(2, 0);
   lcd.print(F("Cooking Done"));
   
   while(!(shortButtonPressed || longButtonPressed))
   {
     Input = measuretemp();
     lcd.setCursor(4,1);
     lcd.print(int(Input/10) % 10);
     lcd.print(int(Input) % 10);
     lcd.print(F("."));
     lcd.print(int(Input*10) % 10);
     lcd.print(F(" C"));
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
// Short for Time
// Long for changing value
// ************************************************
void Tune_Sp()
{
   lcd.print(F("Set Temperature:"));
   while(true)
   {
      button();
      if (change){
        Setpoint = map(analogRead(PotPin), 0, 1023, 15, 99);
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
         opState = SETT;
         change=false;
         return;
      }

      if (!change & (millis() - lastButtonTime) > timeout)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         change=false;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(int(Setpoint/10) % 10);
      lcd.print(int(Setpoint) % 10);
      lcd.print(F("C"));
   }
}

// ************************************************
// Time Entry State
// Potentiometer to change Time
// Short for PID params
// Long for changing value
// ************************************************
void Tune_T()
{
   lcd.print(F("Set Time:"));
   while(true)
   {
      button();
      if (change){
        Time = 2*floor(map(analogRead(PotPin), 0, 1023, 2, 1440)/2);
        StartTime=millis();
        lcd.setCursor(15, 1);
        lcd.print("#");
      };        
      if (longButtonPressed)
      {
         change = !change;
         return;
      }
      if (shortButtonPressed)
      {
         opState = TUNE_P;
         change=false;
         return;
      }

      if (!change & (millis() - lastButtonTime) > timeout)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         change=false;
         return;
      }
     lcd.setCursor(0,1);
     if(int(Time/1000)>0){
       lcd.print(int(Time/1000) % 10);
     }else{
       lcd.print(F(" "));
     }
     if(int(Time/100)>0){
       lcd.print(int(Time/100) % 10);
     }else{
       lcd.print(F(" "));
     }
     if(int(Time/10)>0){
       lcd.print(int(Time/10) % 10);
     }else{
       lcd.print(F(" "));
     }
     lcd.print(int(Time/1) % 10);   
     lcd.print(F("min"));
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
        Kp = map(analogRead(PotPin), 0, 1023, 100, 2000);
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
      if (!change & (millis() - lastButtonTime) > timeout)  // return to RUN after 3 seconds idle
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
        Ki = map(analogRead(PotPin), 0, 1023, 100, 1000)/1000.0;
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
      if (!change & (millis() - lastButtonTime) > timeout)  // return to RUN after 3 seconds idle
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
        Kd = map(analogRead(PotPin), 0, 1023, 50, 200)/1000.0;
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
         opState = AUTO;
         change=false;
         return;
      }
      if (!change & (millis() - lastButtonTime) > timeout)  // return to RUN after 3 seconds idle
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
// Derivative Tuning State
// Potentiometer to change Kd
// Short for RUN
// Long for changing value
// ************************************************
void Auto()
{
   lcd.print(F("Autotune"));
   while(true)
   {
      button();  
      if (longButtonPressed)
      {
         if(!tuning && abs(Input - Setpoint) < 1){
           StartAutoTune();
           opState = RUN;
           TimerOn=false;
         }
         return;
      }
      if (shortButtonPressed)
      {
         opState = RUN;
         return;
      }
      if (!change & (millis() - lastButtonTime) > timeout)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         change=false;
         return;
      }
      lcd.setCursor(0,1);
      if(tuning){
      lcd.print("On");
      }else{
      lcd.print("Off");
      }  
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
   lcd.setCursor(5,0);
   lcd.print(int(Setpoint/10) % 10);
   lcd.print(int(Setpoint) % 10);
   lcd.print(F("C"));

   lcd.setCursor(8,1);
   lcd.print(F("/"));   
   if(int(Time/1000)>0){
     lcd.print(int(Time/1000) % 10);
   }
   if(int(Time/100)>0){
     lcd.print(int(Time/100) % 10);
   }
   if(int(Time/10)>0){
     lcd.print(int(Time/10) % 10);
   }
   lcd.print(int(Time/1) % 10);   
   lcd.print(F("min"));

   SaveParameters();
   myPID.SetTunings(Kp,Ki,Kd);
   while(true)
   {
      button();
      if (shortButtonPressed)
      {
           opState = SETP;
           return;
      }
      if (longButtonPressed)
      {
         TimerOn=!TimerOn;
         if(TimerOn){StartTime=millis();}
         return;
      }

      DoControl();
      
      lcd.setCursor(0,0);
      lcd.print(int(Input/10) % 10);
      lcd.print(int(Input) % 10);
      lcd.print(F("."));
      lcd.print(int(Input*10) % 10);
      lcd.print(F("/"));
      
      float pct = map(Output, 0, WindowSize, 0, 100);
      lcd.setCursor(10,0);
      lcd.print(int(pct/100) % 10);
      lcd.print(int(pct/10) % 10);
      lcd.print(int(pct) % 10);
      lcd.print(F("%"));

      if(TimerOn){
        Minutes=(millis()-StartTime)/60000;
        if(Minutes>=Time)
        {
          opState = DONE;
          TimerOn=false;
          return;
        }
        if(int(Minutes/1000)>0){
          lcd.setCursor(4,1);
          lcd.print(int(Minutes/1000) % 10);
        }
        if(int(Minutes/100)>0){
          lcd.setCursor(5,1);
          lcd.print(int(Minutes/100) % 10);
        }
        if(int(Minutes/10)>0){
          lcd.setCursor(6,1);
          lcd.print(int(Minutes/10) % 10);
        }
        lcd.setCursor(7,1);
        lcd.print(int(Minutes) % 10);
      } else {
        lcd.setCursor(0,1);
        lcd.print(F("TimerOff"));
      }

      lcd.setCursor(15,0);
      if (tuning)
      {
        lcd.print("T");
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
  Minutes=(millis()-StartTime)/60000;

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
  
  // Time Proportional relay state is updated regularly via Timer interrupt.
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
   if (Time != EEPROM_readDouble(TimeAddress))
   {
      EEPROM_writeDouble(TimeAddress, Time);
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
   Time = EEPROM_readDouble(TimeAddress);
   
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
   if (isnan(Time))
   {
     Time = 720;
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
