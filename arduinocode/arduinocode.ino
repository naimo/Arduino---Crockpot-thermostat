#include <LiquidCrystal.h>
#define NUMSAMPLES 5

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 1, 2, 3, 4);

// general variables
int potPin = A2;
int thermPin = A1;
int pushPin = 10;
int relayPin = 6;
double Temperature;

// classic thermostat menu/mode variables
double ClassicSetPoint=15;
double TempClassicSetPoint = 15;

// menu setup
int menu = 0;
int submenu = 0;
boolean newdisplay = true;                         // if display has been cleared, next menu/mode will refresh
int buttonState;
int lastButtonState = LOW;
long lastButtonTime = 0;
long longPressTime = 500;
boolean shortButtonPressed = false;
boolean longButtonPressed = false;
boolean changeValue = false;
int mode = 1;

// thermistor setup
int nominalR = 10000;
int nominalT = 25;
int numsamp = NUMSAMPLES;
int bcoeff = 3950;
int seriesR = 10000;
int samples[NUMSAMPLES];

void setup() {
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
          menu++;
          shortButtonPressed = false;
          lcd.clear();
          newdisplay = true;
      }
      if (longButtonPressed){
        longButtonPressed = false;
        menu=mode;
        submenu=1;
        lcd.clear();
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
              lcd.print(int(TempClassicSetPoint));              
            }
            if (changeValue){
              lcd.setCursor(11, 1);
              TempClassicSetPoint=acquiresettemp();
              lcd.print(int(TempClassicSetPoint));
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
              ClassicSetPoint=TempClassicSetPoint;
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
        lcd.print("Dummy Menu");
      }
      if (shortButtonPressed){
          shortButtonPressed = false;
          lcd.clear();
          menu++;
          newdisplay = true;
      }
      if (longButtonPressed){
        longButtonPressed = false;
      }
      break;      
    
    
    default: 
      menu=1;
      break;
  }
}


void classicthermostat(){
  Temperature = measuretemp();
    if (Temperature < ClassicSetPoint)
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
    lcd.print(int(Temperature));
    lcd.setCursor(5, 0);
    lcd.print(int(ClassicSetPoint));
  }
}

double acquiresettemp(){
  return map(analogRead(potPin), 0, 1023, 99, 15); 
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
