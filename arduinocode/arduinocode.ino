#include <LiquidCrystal.h>
#include <PID_v1.h>
#define relayPin 6
#define NUMSAMPLES 5

double PIDSetpoint, realtemp, Output;
PID myPID(&realtemp, &Output, &PIDSetpoint,2,5,1, DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 1, 2, 3, 4);

int potPin = A2;
int val = 0;
int thermPin = A1;
int pushPin=10;

double classicSetPoint = 15;
int classicDwellTime = 0;
long classicStartTime = 0;
double tempSetPoint = 15;
double tempDwellTime = 0;
boolean changeValue = false;

int menu = 0;
int submenu = 0;
int mode = 1;
int buttonState;  
int lastButtonState = LOW;
long lastButtonTime = 0;
long longPressTime = 500;
long lastDebounceTime = 0;
long debounceDelay = 50;  
boolean newdisplay = true;                         // test if display has been cleared
boolean shortButtonPressed = false;
boolean longButtonPressed = false;

int nominalR = 10000;
int nominalT = 25;
int numsamp = NUMSAMPLES;
int bcoeff = 3950;
int seriesR = 10000;
int samples[NUMSAMPLES];

void setup() {
  windowStartTime = millis();
  PIDSetpoint = 100;
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);
  
  lcd.begin(16, 2);
  pinMode(relayPin, OUTPUT);
  pinMode(pushPin, INPUT);
}

void loop() {
  startmenu();
  
  switch (mode) {
    case 1:
      classicthermostat();
      break;
    case 2:
      PIDthermostat();
      break;    
  }
  delay(50);
}

void startmenu(){
  buttonState = digitalRead(pushPin);
  if (buttonState != lastButtonState) {
    if (buttonState == LOW) {
      if (millis()-lastButtonTime < longPressTime){
        shortButtonPressed = true;
      }else{
        longButtonPressed = true;
      }
    } 
    lastButtonState = buttonState;
    lastButtonTime = millis();
  }
  
  switch (menu) {
    case 0:
      if (shortButtonPressed){
          lcd.clear();
          menu++;
          shortButtonPressed = false;
          newdisplay = true;
      }
      if (longButtonPressed){
        longButtonPressed = false;
        menu=mode;
        submenu=1;
        newdisplay = true;
      }      
      break;
    
    
    case 1:
      if (submenu == 0){
        if (newdisplay){
          newdisplay = false;
          lcd.setCursor(0, 0);
          lcd.print("Thermostat");
        }
        if (shortButtonPressed){
          shortButtonPressed = false;
          lcd.clear();
          menu++;
          newdisplay = true;
        }
        if (longButtonPressed){
          longButtonPressed = false;        
          submenu=1;    
          newdisplay = true;
        }
      }else{
        switch (submenu) {
          case 1:
            if (newdisplay){
              newdisplay = false;
              lcd.setCursor(0, 0);
              lcd.print("Thermostat");
              lcd.setCursor(0, 1);
              lcd.print("Setpoint");
              lcd.setCursor(11, 1);
              lcd.print(int(tempSetPoint));              
            }
            if (changeValue){
              lcd.setCursor(11, 1);
              tempSetPoint=acquiresettemp();
              lcd.print(int(tempSetPoint));
            }            
            if (shortButtonPressed){
              shortButtonPressed = false;
              lcd.clear();
              submenu++;
              newdisplay = true;
              changeValue = false;
           }
            if (longButtonPressed){
              longButtonPressed = false;
              changeValue=!changeValue;
            }
            break;
//          case 2:
//            if (newdisplay){
//              newdisplay = false;
//              lcd.setCursor(0, 0);
//              lcd.print("Thermostat");
//              lcd.setCursor(0, 1);
//              lcd.print("Time (min)");
//            }
//            lcd.setCursor(11, 1);
//            lcd.print(acquiremin());
//            if (shortButtonPressed){
//              shortButtonPressed = false;
//              lcd.clear();
//              submenu++;
//              tempDwellTime=acquiremin();
//              newdisplay = true;
//            }
//            break;
          case 2:
            if (newdisplay){
              newdisplay = false;
              lcd.setCursor(0, 0);
              lcd.print("Thermostat");
              lcd.setCursor(0, 1);
              lcd.print("Accept");
            }                      
            if (shortButtonPressed){
              shortButtonPressed = false;
              lcd.clear();
              submenu++;
              newdisplay = true;
            }
            if (longButtonPressed){
              longButtonPressed = false;        
              menu=0;
              submenu=0;  
              mode=1;  
              lcd.clear();
              newdisplay = true;
              classicSetPoint=tempSetPoint;
//              classicDwellTime=tempDwellTime;      
            }
          case 3:
            if (newdisplay){
              newdisplay = false;
              lcd.setCursor(0, 0);
              lcd.print("Thermostat");
              lcd.setCursor(0, 1);
              lcd.print("Back");
            }
            if (shortButtonPressed){
              shortButtonPressed = false;
              lcd.clear();
              submenu++;
              newdisplay = true;
            }
            if (longButtonPressed){
              longButtonPressed = false;        
              menu=0;
              submenu=0;    
              lcd.clear();
              newdisplay = true;
            }             
            break;
          default: 
            submenu=1;
            break;
        }
      }                
      break;
    
    
    case 2:
      if (newdisplay){
        newdisplay = false;
        lcd.setCursor(0, 0);
        lcd.print("PID Thermostat");
      }
      if (shortButtonPressed){
          shortButtonPressed = false;
          lcd.clear();
          menu++;
          newdisplay = true;
      }
      if (longButtonPressed){
        longButtonPressed = false;
        mode=2;
        menu=0;
        lcd.clear();       
        newdisplay = true;
      }
      break;      
    
    
    default: 
      menu=1;
      break;
  }
}

  
void PIDthermostat(){
  PIDSetpoint = acquiresettemp();
  realtemp = measuretemp();
  myPID.Compute();
  unsigned long now = millis();
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(Output > now - windowStartTime){
    digitalWrite(relayPin,HIGH);
    lcd.setCursor(15, 1);
    lcd.print("#");    
  }else{
    digitalWrite(relayPin,LOW);
    lcd.setCursor(15, 0);
    lcd.print(" ");     
  }
  if (menu == 0) {
    if (newdisplay){
      newdisplay = false;
      lcd.setCursor(0, 0);
      lcd.print("Set:");
      lcd.setCursor(0, 1);
      lcd.print("Real:");
    }
    lcd.setCursor(6, 1);
    lcd.print(int(realtemp));
    lcd.setCursor(5, 0);
    lcd.print(int(PIDSetpoint));
  }  
}

void classicthermostat(){
//  classicSetPoint = acquiresettemp();
  realtemp = measuretemp();
    if (realtemp < classicSetPoint)
    {
      digitalWrite(relayPin, HIGH);
      lcd.setCursor(15, 1);
      lcd.print("#");      
    }
  else
    {
      digitalWrite(relayPin, LOW); 
      lcd.setCursor(15, 1);
      lcd.print(" ");      
    }
  if (menu == 0) {
    if (newdisplay){
      newdisplay = false;
      lcd.setCursor(0, 0);
      lcd.print("Set:");
      lcd.setCursor(0, 1);
      lcd.print("Real:");
    }
    lcd.setCursor(6, 1);
    lcd.print(int(realtemp));
    lcd.setCursor(5, 0);
    lcd.print(int(classicSetPoint));
  }
}

double acquiresettemp(){
  val = analogRead(potPin);
  return map(val, 0, 1023, 99, 15); 
}

int acquirehour(){
  val = analogRead(potPin);
  return map(val, 0, 1023, 24, 0); 
}

int acquiremin(){
  val = analogRead(potPin);
  return map(val, 0, 1023, 59, 0); 
}
  
double measuretemp(){
  uint8_t i;
  float average;
  for (i=0; i< numsamp; i++) {
   samples[i] = analogRead(thermPin);
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
