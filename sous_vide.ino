#include <PID_v1.h>
#include <SimpleTimer.h>
#include <OneWire.h>
#include <LiquidCrystal.h>

// UI states
#define STATE_INIT 0
#define STATE_TEMPERATURE 1
#define STATE_SET_TARGET 2
#define STATE_SET_DURATION 3
#define STATE_MAX 3
int state = STATE_INIT;
int prevState = STATE_INIT;

// Cooking states
#define CSTATE_IDLE 0
#define CSTATE_INITIALIZE 1
#define CSTATE_PREHEAT 2
#define CSTATE_RUNNING 3
#define CSTATE_DONE 4
int cState = CSTATE_IDLE;

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// define some values used by the panel and buttons
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

// read the buttons
int read_LCD_buttons()
{
 adc_key_in = analogRead(0);      // read the value from the sensor 
 // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
 // we add approx 50 to those values and check to see if we are close
 if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
 if (adc_key_in < 50)   return btnRIGHT;  
 if (adc_key_in < 195)  return btnUP; 
 if (adc_key_in < 380)  return btnDOWN; 
 if (adc_key_in < 555)  return btnLEFT; 
 if (adc_key_in < 790)  return btnSELECT;   
 return btnNONE;  // when all others fail, return this...
}

int DS18S20_Pin = 2; //DS18S20 Signal pin on digital 2

//Temperature chip i/o
OneWire ds(DS18S20_Pin); // on digital pin 2

//Celsius to Fahrenheit conversion
double Fahrenheit(double celsius)
{
        return 1.8 * celsius + 32;
}

float getTemp(){
 //returns the temperature from one DS18S20 in DEG Celsius
 byte data[12];
 byte addr[8];

 if ( !ds.search(addr)) {
   //no more sensors on chain, reset search
   ds.reset_search();
   return -1000;
 }

 if ( OneWire::crc8( addr, 7) != addr[7]) {
   Serial.println("CRC is not valid!");
   return -1000;
 }

 if ( addr[0] != 0x10 && addr[0] != 0x28) {
   Serial.print("Device is not recognized");
   return -1000;
 }

 ds.reset();
 ds.select(addr);
 ds.write(0x44,1); // start conversion, with parasite power on at the end

 byte present = ds.reset();
 ds.select(addr);  
 ds.write(0xBE); // Read Scratchpad

 
 for (int i = 0; i < 9; i++) { // we need 9 bytes
  data[i] = ds.read();
 }
 
 ds.reset_search();
 
 byte MSB = data[1];
 byte LSB = data[0];

 float tempRead = ((MSB << 8) | LSB); //using two's compliment
 float TemperatureSum = tempRead / 16;
 
 return TemperatureSum;
 
}

int prevStrLength = 0;
void log(char* str) {
  if (strlen(str) < prevStrLength) {
    lcd.clear();
  }
  prevStrLength = strlen(str);
  lcd.setCursor(0, 0);
  lcd.print(str);
}

SimpleTimer t;
int lcdButtonState = btnNONE;
int prevLcdButtonState = btnNONE;
int updateButtonState() {
  prevLcdButtonState = lcdButtonState;
  lcdButtonState = read_LCD_buttons();
  if (prevLcdButtonState != lcdButtonState) {
    Serial.println("Button pressed");
    return lcdButtonState;
  }
  
//  Serial.println("No button change");
  return btnNONE;
}


double currentTemp = 0.0;
int windowSize = 10000;
double pidOutput;

// ui configurable params
double targetTemp = 110.0;
int duration = 60;
int relayPin = 3;



static void checkTemp() {
//  Serial.println("checkTemp");
  currentTemp = Fahrenheit(getTemp());
}

void setRelayPower(boolean isOn) {
  digitalWrite(relayPin, isOn);
}

unsigned long windowStartTime;

PID mPID(&currentTemp, &pidOutput, &targetTemp, 2000, 0.25, 0, DIRECT);

static void updateCookState() {
  unsigned long now = millis();
  
  Serial.println(currentTemp);
  // update cook state
  switch (cState) {
    case CSTATE_IDLE:
      break;
    case CSTATE_INITIALIZE:
      mPID.SetOutputLimits(0, windowSize);
      mPID.SetMode(AUTOMATIC);
      windowStartTime = millis();
      cState = CSTATE_PREHEAT;
      break;
    case CSTATE_PREHEAT:
      mPID.Compute();
      Serial.println(windowStartTime);
      Serial.println(pidOutput);
      
      if (now - windowStartTime > windowSize) { //time to shift the Relay Window
        windowStartTime += windowSize;
      }
      
      if (pidOutput > now - windowStartTime) {
        setRelayPower(true);
      } else {
        setRelayPower(false);
      }
      
      if (abs(currentTemp - targetTemp) < 0.5) {
        cState = CSTATE_RUNNING;
      }
      
      break;
    case CSTATE_RUNNING:
      mPID.Compute();
      Serial.println(windowStartTime);
      Serial.println(pidOutput);
      
      if (now - windowStartTime > windowSize)
      { //time to shift the Relay Window
        windowStartTime += windowSize;
      }
      
      if (pidOutput > now - windowStartTime) {
        setRelayPower(true);
      } else {
        setRelayPower(false);
      }
      break;
    case CSTATE_DONE:
      break;
  }    
}

void setup()
{
  Serial.begin(115200);
  Serial.println("DHT11 TEST PROGRAM ");
  //Serial.print("LIBRARY VERSION: ");
  //Serial.println(DHT11LIB_VERSION);
  Serial.println();
  lcd.begin(16,2);
  log("Initializing...");
 
  pinMode(relayPin, 3);
  // setup button detection loop
  t.setInterval(100, checkTemp);
  t.setInterval(1000, updateCookState);
}

bool isRunning = false;
void loop()
{
  static boolean didTransition = false;
  if (state != prevState) {
    didTransition = true;
    lcd.clear();
  } else {
    didTransition = false; 
  }
  prevState = state;
  
  // update variables
  int buttonState = updateButtonState();

  // update state
  switch(state) {
    case STATE_INIT:
      log("Sous Vide v0.1");
      
      lcd.setCursor(0, 1);
      switch (cState) {
        case CSTATE_IDLE:
          lcd.print("IDLE");
          break;
        case CSTATE_INITIALIZE:
          lcd.print("INIT");
          break;
        case CSTATE_PREHEAT:
          lcd.print("PREHEAT");
          break;
        case CSTATE_RUNNING:
          lcd.print("RUNNING");
          break;
        case CSTATE_DONE:
          lcd.print("DONE");
          break;
      }        
      
      if (buttonState == btnSELECT) {
        //TODO: validate constraints
        cState = CSTATE_INITIALIZE;
        updateCookState(); // trigger manual update to get thing started
      }
      break;
    case STATE_TEMPERATURE:
      log("Current Temp:");
      lcd.setCursor(0, 1);
      lcd.print(currentTemp, 1);
      lcd.print((char)223);
      lcd.print("F");
      break;
    case STATE_SET_TARGET:
      log("Set Target:");
      lcd.setCursor(0, 1);
      lcd.print(targetTemp, 1);
      lcd.print("F");
      
      if (read_LCD_buttons() == btnUP)
        targetTemp = min(targetTemp + 0.5, 170);
      else if (read_LCD_buttons() == btnDOWN)
        targetTemp = max(targetTemp - 0.5, 100);
      break;
      
    case STATE_SET_DURATION:
      log("Set Duration:");
      lcd.setCursor(0, 1);
      lcd.print(duration, 1);
      lcd.print("M");
      
      if (read_LCD_buttons() == btnUP)
        duration = min(duration + 5, 720);
      else if (read_LCD_buttons() == btnDOWN)
        duration = max(duration - 5, 5);
      break;
      
    default:
      log("STATE_DEFAULT");
      break;
  }

  if (buttonState == btnRIGHT)
    state = min(state + 1, STATE_MAX);
  else if(buttonState == btnLEFT)
    state = max(state - 1, 0);
  t.run();
  delay(100);
}
//
// END OF FILE
//
