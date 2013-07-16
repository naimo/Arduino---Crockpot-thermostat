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
double ThermostatInterval = 5000;
double LastThermostat = 0;

// classic thermostat menu/mode variables
double ClassicSetPoint=43;
double TempClassicSetPoint = 15;

// yogurt variables
double SanitizeTemp = 83; // degrees C
double StarterHoldTemp = 43; // degrees C
double IncubTemp = 40; // degrees C
double IncubDuration = 36000; // seconds
double IncubStartTime;
int YogStep = 1;

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
int mode = 3;

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
  button(); 

  switch (mode) {
  case 1:
    basicthermostat();
    break;
  case 2:
    yogurt();
    break;
  case 3:
    shortyogurt();
    break;
  }

  startmenu();
  if (digitalRead(relayPin)==HIGH){
    lcd.setCursor(15, 1);
    lcd.print("#");      
  }
  else{
    lcd.setCursor(15, 1);
    lcd.print(" ");      
  }
  delay(50);
}

void thermostat(){
  Temperature = measuretemp();
  if (millis()>LastThermostat + ThermostatInterval){
    LastThermostat=millis();
    if (Temperature < ClassicSetPoint){
      digitalWrite(relayPin, HIGH);
    }else{
      digitalWrite(relayPin, LOW);
    }
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

void button(){
  buttonState = digitalRead(pushPin);
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

void basicthermostat(){
  thermostat();
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

void yogurt(){
  switch(YogStep){
  case 1:
    if (menu == 0) {
      if (newdisplay){
        newdisplay = false;
        lcd.setCursor(0, 0);
        lcd.print("Heat to");
        lcd.setCursor(0, 1);
        lcd.print("Temp");
      }
      lcd.setCursor(6, 1);
      lcd.print(int(Temperature));
      lcd.setCursor(9, 0);
      lcd.print(int(SanitizeTemp));
    }
    Temperature = measuretemp();
    if (Temperature >= SanitizeTemp){
      YogStep++;
      lcd.clear();
      newdisplay = true;
      digitalWrite(relayPin, LOW); 
    }
    break;

  case 2:
    if (menu == 0) {
      if (newdisplay){
        newdisplay = false;
        lcd.setCursor(0, 0);
        lcd.print("Cool to");
        lcd.setCursor(0, 1);
        lcd.print("Temp");
      }
      lcd.setCursor(6, 1);
      lcd.print(int(Temperature));
      lcd.setCursor(9, 0);
      lcd.print(int(StarterHoldTemp));
    }
    Temperature = measuretemp();
    if (int(Temperature) <= int(StarterHoldTemp)){
      YogStep++;
      lcd.clear();
      ClassicSetPoint = StarterHoldTemp;
      newdisplay = true;
    }
    break;

  case 3:
    if (menu == 0) {
      if (newdisplay){
        newdisplay = false;
        lcd.setCursor(0, 0);
        lcd.print("Add starter");
        lcd.setCursor(0, 1);
        lcd.print("Temp");
      }
      lcd.setCursor(6, 1);
      lcd.print(int(Temperature));
      if (shortButtonPressed || longButtonPressed){
        shortButtonPressed = false;
        longButtonPressed = false;
        YogStep++;
        lcd.clear();
        ClassicSetPoint = IncubTemp;
        IncubStartTime = millis();
        newdisplay = true;
      }        
    }
    thermostat();
    break;

  case 4:
    if (menu == 0) {
      if (newdisplay){
        newdisplay = false;
        lcd.setCursor(0, 0);
        lcd.print("Incub   /  h");
        lcd.setCursor(9, 0); 
        lcd.print(int(IncubDuration/3600));    
        lcd.setCursor(0, 1);
        lcd.print("Temp");
      }
      lcd.setCursor(6, 0); 
      lcd.print(int((millis()-IncubStartTime)/1000/3600));      
      lcd.setCursor(6, 1);
      lcd.print(int(Temperature));
    }
    thermostat();
    if (millis() - IncubStartTime > 1000*IncubDuration){
      YogStep++;
      lcd.clear();
      digitalWrite(relayPin, LOW); 
      newdisplay = true;
    }
    break;

  case 5:
    if (menu == 0) {
      if (newdisplay){
        newdisplay = false;
        lcd.setCursor(0, 0);
        lcd.print("Ready");
        lcd.setCursor(0, 1);
        lcd.print("Temp");
      }
      Temperature = measuretemp();
      lcd.setCursor(6, 1);
      lcd.print(int(Temperature));
    }
    break;      
  }
}

void shortyogurt(){
  switch(YogStep){
  case 1:
    if (menu == 0) {
      if (newdisplay){
        newdisplay = false;
        lcd.setCursor(0, 0);
        lcd.print("Reaching");
        lcd.setCursor(0, 1);
        lcd.print("Temp");
      }
      lcd.setCursor(6, 1);
      lcd.print(int(Temperature));
      lcd.setCursor(9, 0);
      lcd.print(int(StarterHoldTemp));
    }
    thermostat();
    if (int(Temperature) == int(StarterHoldTemp)){
      YogStep++;
      lcd.clear();
      ClassicSetPoint = StarterHoldTemp;
      newdisplay = true;
    }
    break;

  case 2:
    if (menu == 0) {
      if (newdisplay){
        newdisplay = false;
        lcd.setCursor(0, 0);
        lcd.print("Add starter");
        lcd.setCursor(0, 1);
        lcd.print("Temp");
      }
      lcd.setCursor(6, 1);
      lcd.print(int(Temperature));
      if (shortButtonPressed || longButtonPressed){
        shortButtonPressed = false;
        longButtonPressed = false;
        YogStep++;
        lcd.clear();
        ClassicSetPoint = IncubTemp;
        IncubStartTime = millis();
        newdisplay = true;
      }        
    }
    thermostat();
    break;

  case 3:
    if (menu == 0) {
      if (newdisplay){
        newdisplay = false;
        lcd.setCursor(0, 0);
        lcd.print("Incub   /  h");
        lcd.setCursor(9, 0); 
        lcd.print(int(IncubDuration/3600));    
        lcd.setCursor(0, 1);
        lcd.print("Temp");
      }
      lcd.setCursor(6, 0); 
      lcd.print(int((millis()-IncubStartTime)/1000/3600));      
      lcd.setCursor(6, 1);
      lcd.print(int(Temperature));
    }
    thermostat();
    if (millis() - IncubStartTime > 1000*IncubDuration){
      YogStep++;
      lcd.clear();
      digitalWrite(relayPin, LOW); 
      newdisplay = true;
    }
    break;

  case 4:
    if (menu == 0) {
      if (newdisplay){
        newdisplay = false;
        lcd.setCursor(0, 0);
        lcd.print("Ready");
        lcd.setCursor(0, 1);
        lcd.print("Temp");
      }
      Temperature = measuretemp();      
      lcd.setCursor(6, 1);
      lcd.print(int(Temperature));
    }
    break;      
  }
}

void startmenu(){
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
        lcd.clear();
        newdisplay = true;
      }
    }
    else{
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
          submenu++;
          lcd.clear();
          newdisplay = true;
        }
        if (longButtonPressed){
          longButtonPressed = false;        
          menu=0;
          submenu=0;  
          mode=1;  
          ClassicSetPoint=TempClassicSetPoint;              
          lcd.clear();
          newdisplay = true;
        }
        break;
      case 3:
        if (newdisplay){
          newdisplay = false;
          lcd.setCursor(0, 0);
          lcd.print("Thermostat");
          lcd.setCursor(0, 1);
          lcd.print("Leave menu");
        }
        if (shortButtonPressed){
          shortButtonPressed = false;
          submenu++;
          lcd.clear();
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
    if (submenu == 0){
      if (newdisplay){
        newdisplay = false;
        lcd.setCursor(0, 0);
        lcd.print("Yogurt");
      }
      if (shortButtonPressed){
        shortButtonPressed = false;
        menu++;
        lcd.clear();
        newdisplay = true;
      }
      if (longButtonPressed){
        longButtonPressed = false;        
        submenu=1;
        lcd.clear();    
        newdisplay = true;
      }
    }
    else{
      switch (submenu) {
      case 1:
        if (newdisplay){
          newdisplay = false;
          lcd.setCursor(0, 0);
          lcd.print("Yogurt");
          lcd.setCursor(0, 1);
          lcd.print("(re)Start");
        }                      
        if (shortButtonPressed){
          shortButtonPressed = false;
          submenu++;
          lcd.clear();
          newdisplay = true;
        }
        if (longButtonPressed){
          longButtonPressed = false;        
          menu=0;
          submenu=0;  
          mode=2;
          YogStep=1;  
          digitalWrite(relayPin, HIGH);
          lcd.clear();
          newdisplay = true;
        }
        break;
        
      case 2:
        if (newdisplay){
          newdisplay = false;
          lcd.setCursor(0, 0);
          lcd.print("Yogurt");
          lcd.setCursor(0, 1);
          lcd.print("Leave menu");
        }
        if (shortButtonPressed){
          shortButtonPressed = false;
          submenu++;
          lcd.clear();
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

  case 3:
    if (submenu == 0){
      if (newdisplay){
        newdisplay = false;
        lcd.setCursor(0, 0);
        lcd.print("ShortYogurt");
      }
      if (shortButtonPressed){
        shortButtonPressed = false;
        menu++;
        lcd.clear();
        newdisplay = true;
      }
      if (longButtonPressed){
        longButtonPressed = false;        
        submenu=1;
        lcd.clear();    
        newdisplay = true;
      }
    }
    else{
      switch (submenu) {
      case 1:
        if (newdisplay){
          newdisplay = false;
          lcd.setCursor(0, 0);
          lcd.print("ShortYogurt");
          lcd.setCursor(0, 1);
          lcd.print("(re)Start");
        }                      
        if (shortButtonPressed){
          shortButtonPressed = false;
          submenu++;
          lcd.clear();
          newdisplay = true;
        }
        if (longButtonPressed){
          longButtonPressed = false;        
          menu=0;
          submenu=0;  
          mode=3;
          YogStep=1;  
          ClassicSetPoint=StarterHoldTemp;
          lcd.clear();
          newdisplay = true;
        }
        break;
        
      case 2:
        if (newdisplay){
          newdisplay = false;
          lcd.setCursor(0, 0);
          lcd.print("ShortYogurt");
          lcd.setCursor(0, 1);
          lcd.print("Leave menu");
        }
        if (shortButtonPressed){
          shortButtonPressed = false;
          submenu++;
          lcd.clear();
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

  default: 
    menu=1;
    break;
  }
}
