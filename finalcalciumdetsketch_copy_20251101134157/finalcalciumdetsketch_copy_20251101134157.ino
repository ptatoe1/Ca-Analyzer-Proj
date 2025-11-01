#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <math.h>
#include "esp32-hal-ledc.h"
#include "Arduino.h"
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

void configureSensor() {
  tsl.setGain(TSL2591_GAIN_HIGH);                 // 428x gain
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);  // longest exposure

}

int readingMode = 0;
uint16_t blankReading = 0;
uint16_t sampleReading = 0;
double absorbance = 0;
bool blanking = false;
bool criticalError = false;
int activity = 0;
const int redPin   = A3;
const int greenPin = A4;
const int bluePin  = A5;
// "channels" are hardware PWM generators inside the ESP32 and you can attach a channel to any pin you want.
// for example, calling ledcWrite(redPin, value) updates the attached channel
// which in turn changes the signal on the pin.
const int redChannel   = 0;
const int greenChannel = 1;
const int blueChannel  = 2;
const int pwmFreq = 5000;
const int pwmRes  = 8;  // (0–255 resolution, standard RGB)
unsigned long lastUpdate = 0;
int step = 0;
int state = 0;
unsigned long pressStart = 0;
bool longPressTriggered = false;
const unsigned long longPressThreshold = 1500; // ms, this is for the rollback function


//stepper struct
struct Stepper {
  int stepPin;
  int dirPin;
  int enPin;
  int limitPin;
  int stepsRemaining;
  bool dosing;
};

Stepper step1 = {3, 4, 2, 5, 0, false};   // leftmost pump. (STEP, DIR, EN, LIMIT)
Stepper step2 = {12, 11, 13, 6, 0, false}; // EN not wired, use -1
Stepper step3 = {A0, A1, A2, 7, 0, false};

//button struct
struct Button {
  int pin;
  int lastState;
};

Button measureBtn = {10, HIGH}; //also serves as a refill button
Button blankBtn   = {9, HIGH};

//stepper timing and speed
int STEPS_PER_SEC = 3000;
unsigned long STEP_PERIOD_US = 1000000UL / STEPS_PER_SEC;  //defines the time between steps, the inverse of frequency 
const unsigned int STEP_PULSE_US   = 15; //if this is too short, the motor will simply stutter and not run. This is the logic signal that
//allows current to flow into the motor coils

unsigned long last_step_micros = 0;
bool dirState1 = LOW;  // start forward
bool dirState2 = HIGH;  // start forward
bool dirState3 = LOW;  // start forward

//LCD states for startup sequence
unsigned long lcd_timer = 0;
int lcd_phase = 0;

//TSL2591 is the name of the light sensor from adafruit
void setup() {
  Serial.begin(115200);
  if (!tsl.begin()) {
    Serial.println("No TSL2591 found ... check wiring!");
    while (1);
  }
  Serial.println("TSL2591 found");
  configureSensor();

  // Stepper 1
  pinMode(step1.enPin, OUTPUT);
  pinMode(step1.stepPin, OUTPUT);
  pinMode(step1.dirPin, OUTPUT);
  pinMode(step1.limitPin, INPUT_PULLUP); //pullup to prevent voltage floating and false triggers

  // Stepper 2
  if (step2.enPin != -1) pinMode(step2.enPin, OUTPUT);
  pinMode(step2.stepPin, OUTPUT);
  pinMode(step2.dirPin, OUTPUT);
  pinMode(step2.limitPin, INPUT_PULLUP);

  // Stepper 3
  pinMode(step3.enPin, OUTPUT);
  pinMode(step3.stepPin, OUTPUT);
  pinMode(step3.dirPin, OUTPUT);
  pinMode(step3.limitPin, INPUT_PULLUP);
  // Buttons
  pinMode(measureBtn.pin, INPUT_PULLUP);
  pinMode(blankBtn.pin, INPUT_PULLUP);

  // Init outputs
  digitalWrite(step1.stepPin, LOW);
  digitalWrite(step1.dirPin, dirState1);
  digitalWrite(step1.enPin, LOW);  // enable driver
  digitalWrite(step2.stepPin, LOW);
  digitalWrite(step2.dirPin, dirState2);
  digitalWrite(step3.stepPin, LOW);
  digitalWrite(step3.dirPin, dirState3);
  digitalWrite(step3.enPin, LOW);  // enable driver

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("SCALe starting..."); //Spectrophotometric Calcium Analyzer for Laboratory Evaluation
  lcd_timer = millis();
  lcd_phase = 0;

  //RGB LED
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  // attaching pins to PWM channels
  ledcAttachChannel(redPin, pwmFreq, pwmRes, redChannel);
  ledcAttachChannel(greenPin, pwmFreq, pwmRes, greenChannel);
  ledcAttachChannel(bluePin, pwmFreq, pwmRes, blueChannel);

  last_step_micros = micros();
}

static inline void stepPulse(int pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(STEP_PULSE_US); //high for a duration given by STEP_PULSE_US Variable 
  digitalWrite(pin, LOW);
}

void lcdMessage(const char *line1, const char *line2) { //useful function for quick and easy message prints
  lcd.setCursor(0, 0);
  lcd.print("                ");  // clear row
  lcd.setCursor(0, 0);
  lcd.print(line1);

  lcd.setCursor(0, 1);
  lcd.print("                ");  // clear row
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void loop() {
  //rgb cycle because rainbows are fun and pretty
  if (activity == 0 ) { //idle
    if (millis() - lastUpdate > 10) { // every 10 ms
      lastUpdate = millis();
      disableMotors();
      doLEDCycleStep(); 
      step++;
      if (step > 255 * 3) step = 0;
    }
  }
  
  
  

  unsigned long now_ms = millis();
  
  //startup sequence
  if (lcd_phase == 0 && now_ms - lcd_timer > 3000) {
    lcdMessage("Provide blank", "solution");
    lcd_timer = now_ms;
    lcd_phase = 1;
  } 
  else if (lcd_phase == 1 && now_ms - lcd_timer > 2000) {  //uses millis timing instead of delay so it doesn’t pause the rest of loop
    if(criticalError == false){
    lcdMessage("Device ready", "");
    }
    else {
    lcdMessage("Change reagents", "");
    }
    lcd_phase = 2;
  }


// ------- debounced measure button handling -------
// this essentially stops listening for changes in button input if insufficient time has passeed
const unsigned long debounceDelay = 50; // ms
static int stableState = HIGH;          // last debounced stable state
static int lastReading = HIGH;          // last raw reading
static unsigned long lastDebounceTime = 0;

int reading = digitalRead(measureBtn.pin);

//debounce filter
if (reading != lastReading) {
  lastDebounceTime = millis();  // reset timer whenever raw state changes
}
lastReading = reading;

//only act if state has been stable long enough
if ((millis() - lastDebounceTime) > debounceDelay) {
  if (reading != stableState) { //if the current reading is different from the state it was stably in
    stableState = reading; //change the state 

    // pressed down
    if (stableState == LOW) {
      pressStart = millis();
      longPressTriggered = false;
    }

    // released
    else if (stableState == HIGH) {
      unsigned long pressDuration = millis() - pressStart; //tracks how much time has passed since the button was first pressed

      if (pressDuration < longPressThreshold && !longPressTriggered) {
        // Short press: dosing
        if (blankReading == 0) {
          lcd.setCursor(0,0);
          lcd.print("Error:           ");
          scrollingMessageLine("Please blank solution", 1);
          delay(400);
          if(!criticalError) lcdMessage("Device ready", "");
          else lcdMessage("Change reagents", "");
        } else {
          dosePVSW(300);
          doseSample(300);
          doseSolvent(3000);
          readingMode = 1;
        }
      }
    }
  }
}

// while held (stable LOW), check for long press
if (stableState == LOW) {
  unsigned long pressDuration = millis() - pressStart;
  if (pressDuration >= longPressThreshold && !longPressTriggered) {
    lcdMessage("Refilling...", "");
    Serial.println("Starting refill sequence");
    homingSequence();
    lcdMessage("Device ready", "");
    longPressTriggered = true;   // latch
  }
}



  //sample blank
  int blank_state = digitalRead(blankBtn.pin);
  if (blankBtn.lastState == HIGH && blank_state == LOW) {
    blank(); //SET VOLUME HERE
    readingMode = 0; //blank
  }

  blankBtn.lastState = blank_state;


  // --- closing sequence ---
  if ((step1.stepsRemaining == 0 && step1.dosing || blanking == true) && criticalError == false) {
    
    step1.dosing = false;
    step2.dosing = false;
    step3.dosing = false;
    //make sure nothing is marked as running
    if(blanking == false){
      lcdMessage("Complete!", "");
      Serial.println("Dose complete.");
      delay(1000);
      lcdMessage("Taking reading...", "");
      Serial.println("Taking reading.");
    }
    if(blanking == true){
      Serial.println("Blanking");
      lcdMessage("Blanking...", "");
    }
    blanking = false;
    uint16_t reading = readLight(); //reads the same way, a raw light value regardless of mode

    if(readingMode == 0){ //readingMode = 0 signifies blanking
      lcd.setCursor(0, 0);
      lcd.print("Blank reading:                   ");
      lcd.setCursor(0, 1);
      lcd.print(reading); 
      Serial.print(reading);
      blankReading = reading;
    }
    else{
      
      sampleReading = reading;
      Serial.print("Sample: ");
      Serial.println(sampleReading);
      Serial.print("Blank: ");
      Serial.println(blankReading);
      Serial.print("Ratio: ");
      if (blankReading != 0) {
        float ratio = (float)sampleReading / blankReading;
        float absorbance = -log10(ratio); //converts the absolute light absorbance ratio to absorbance (which is meant to linearize this ratio)

        lcd.setCursor(0, 0);
        lcd.print("Absorbance:           ");
        lcd.setCursor(0, 1);
        lcd.print(absorbance, 3);   // always 3 decimal places
        double calciumMolarity = 0;
        calciumMolarity = (51 * (absorbance * absorbance) - 27500 * absorbance + 17425000) / 25000000.0; // in millimoles
       delay(1000);
        lcd.setCursor(0, 0);
        lcd.print("                   ");   // clear line 1
        lcd.setCursor(0, 1);
        lcd.print("                   ");   // clear line 2

      
        lcd.setCursor(0, 0);
        lcd.print("[Ca2+]:");

        Serial.println(calciumMolarity, 2);
        Serial.println (absorbance);

        lcd.setCursor(0, 1);
        lcd.print(calciumMolarity, 3);    // display concentration
        lcd.print(" +/-0.75mM");    
      }
      else{
        lcdMessage("Error: ", "No blank");
      }
      

    }
    activity = 0; //this is for status light color (rainbow for inactive)
  }
}


//--------------------METHODS------------------------

// All of these functions are blocking on purpose, though in a final build, an emergency shutoff listener would be nice.
void dosePVSW(int microL) {
  enableMotors();
  if (criticalError) return;   // don't start if error present

  activity = 2; //turns led purple
  setLED();
  if (!step1.dosing) {
    step1.dosing = true;
    int steps_per_mL = 39112;  //calibrate this number for volume
    double target_volume_mL = microL / 1000.0; //simply converts the target microL value into miliL
    step1.stepsRemaining = steps_per_mL * target_volume_mL;
    lcdMessage("Dosing PVSW", "");
    Serial.println("PVSW Dose started.");
  }

  while (step1.dosing && step1.stepsRemaining > 0) {
    //  Limit switch check
    if (digitalRead(step1.limitPin) == LOW) {
      criticalError = true;              // latch error
      step1.dosing = false;
      step1.stepsRemaining = 0;
      lcdMessage("Failed (A),", "Check reagents");
      Serial.println("Limit reached, stopped PVSW.");
      break;
    }
//pulses a step if a period duration has passed
    unsigned long now_us = micros();
    if (now_us - last_step_micros >= STEP_PERIOD_US) {
      last_step_micros = now_us;
      stepPulse(step1.stepPin);
      step1.stepsRemaining--;
    }
  }
}


void blank(){
  activity = 1; //this is for status light color (red for blank)
  setLED();
  delay(500);
  blanking = true;
  

}
void doseSolvent(int microL) {
  enableMotors();
  if (criticalError) return;

  activity = 5; //green
  setLED();
  if(!step3.dosing){ 
    step3.dosing = true; 
    int steps_per_mL = 2820;
    double target_volume_mL = microL / 1000.0;
    step3.stepsRemaining = steps_per_mL * target_volume_mL; 
    lcdMessage("Dosing solvent", "");
    Serial.println("Solvent Dose started.");
  }

  while(step3.dosing && step3.stepsRemaining > 0){
    if (digitalRead(step3.limitPin) == LOW) {
      criticalError = true;
      step3.dosing = false;
      step3.stepsRemaining = 0;
      lcdMessage("Failed (C),", "Check reagents");
      Serial.println("Limit reached, stopped solvent.");
      break;
    }

    unsigned long now_us = micros();
    if (now_us - last_step_micros >= STEP_PERIOD_US){
      last_step_micros = now_us;
      stepPulse(step3.stepPin);
      step3.stepsRemaining--;
    }
  }

}

void doseSample(int microL) {
  enableMotors();
  if (criticalError) return;

  activity = 4;
  setLED();
  if(!step2.dosing){ //initialization of volume and step variables. 
    step2.dosing = true; 
    int steps_per_mL = 39112;
    double target_volume_mL = microL / 1000.0;
    step2.stepsRemaining = steps_per_mL * target_volume_mL; //sets target
    lcdMessage("Dosing sample", "");
    Serial.println("Sample Dose started.");
  }

  while(step2.dosing && step2.stepsRemaining > 0){
    if (digitalRead(step2.limitPin) == LOW) {
      criticalError = true;
      step2.dosing = false;
      step2.stepsRemaining = 0;
      lcdMessage("Failed (B),", "Check reagents");
      Serial.println("Limit reached, stopped sample.");
      break;
    }

    unsigned long now_us = micros();
    if (now_us - last_step_micros >= STEP_PERIOD_US){ //how much time has passed since startup vs when the next step should occur
      last_step_micros = now_us;
      stepPulse(step2.stepPin);
      step2.stepsRemaining--;
    }
  }
}

int readLight() {
  activity = 3;
  setLED();
  unsigned long start = millis();
  unsigned long duration = 4000;  // 4 seconds
  uint32_t sumVisible = 0;
  uint16_t count = 0;

  while (millis() - start < duration) {
    uint32_t lum = tsl.getFullLuminosity();  //this is a 32 bit word that stores the ir reading in the first 16 digits and the full reading in the last 16.
    uint16_t ir   = lum >> 16; //this shifts the digits of lum right, since the first 16 digits are ir light
    uint16_t full = lum & 0xFFFF; /*in binary, this is 16 digits of 1's. The & function then compares the bit values through an AND gate and
    keeps the digit if it is a 1 (since 1 AND 1 = 1) while erasing everything in the digits right of it (since x AND 0 = 0). 0 values in the leftover
    digits are kept as 0s (since 0 AND 0 = 0). The result is a masked 32-bit value that keeps the last 16 digits (which is the full spectrum value)*/
    uint16_t visible;
    if (full > ir)
    visible = (full - ir); //visible light is obtained by taking away the infrared light from the total light 
    else
    visible = 0; 

    if (full >= 65500 || ir >= 65500) {
      Serial.println("Sensor saturated!");
    }

    Serial.print("full="); Serial.print(full);
    Serial.print(" ir=");  Serial.print(ir);
    Serial.print(" vis="); Serial.println(visible);

    sumVisible += visible; //adds up the total value of the readings of visible
    count++;

    
  }

  uint16_t avgVisible = sumVisible / count;
  return avgVisible;
}

void doLEDCycleStep() {
  int phase = step / 256;     // which segment (0,1,2)
  int value = step % 256;     // position inside segment

  int r = 0, g = 0, b = 0;

  switch (phase) {
    case 0: // lerping between Red and Green, same concept as in unity
      r = 255 - value;
      g = value;
      b = 0;
      break;

    case 1: // lerping between Green and Blue
      r = 0;
      g = 255 - value;
      b = value;
      break;

    case 2: // lerping between Blue and Red
      r = value;
      g = 0;
      b = 255 - value;
      break;
  }

  ledcWrite(redPin, r); //this function conveniently converts from an rgb value to an equivalent PWM configuration
  ledcWrite(greenPin, g);
  ledcWrite(bluePin, b);

  step++;
  if (step >= 256 * 3) step = 0; // wrap after full rainbow
}

void setLED(){ //to instantly set the LED light a certain color
  char r = 0;
  char g = 0;
  char b = 0;

  switch(activity){
    case 0:
      break;
    case 1: //blanking (red)
      r = 255; g = 0;   b = 0;
      break;
    case 2: //PVSW dosing (purple)
      r = 255; g = 0;   b = 255;
      break;
    case 3: //Light measuring (white)
      r = 255; g = 255; b = 255;
      break;
    case 4: //Sample Dosing (blue)
      r = 0; g = 0; b = 255;
      break;
    case 5: //Solvent Dosing (green)
      r = 0; g = 255; b = 0;
      break;
    default:
      break;
  }

  // assigning rgb to pins
  ledcWrite(redPin, r);
  ledcWrite(greenPin, g);
  ledcWrite(bluePin, b);
}


// line = 0 (top) or 1 (bottom) for a 16x2 LCD
void scrollingMessageLine(String message, int line) {
  int lcdWidth = 16;   // characters per line

  // if message fits just print
  if (message.length() <= lcdWidth) {
    lcd.setCursor(0, line);
    lcd.print(message);
    // pad with spaces to clear leftovers
    for (int i = message.length(); i < lcdWidth; i++) {
      lcd.print(" ");
    }
    return;
  }

  // otherwise scroll the message
  for (int start = 0; start <= message.length() - lcdWidth; start++) {
    lcd.setCursor(0, line);
    lcd.print(message.substring(start, start + lcdWidth));
    delay(200); // adjust speed (ms per shift)
  }
}




void homingSequence() {
  STEPS_PER_SEC = 2500;
  STEP_PERIOD_US = 1000000UL / STEPS_PER_SEC;
  enableMotors();
  bool stopP1 = false;
  bool stopP2 = false;
  bool stopP3 = false;

  //moves backwards toward limit switches
  digitalWrite(step1.dirPin, HIGH);  // backwards
  digitalWrite(step2.dirPin, LOW);
  digitalWrite(step3.dirPin, HIGH);

  //timers to control the pace of each motor
  unsigned long t1 = micros();
  unsigned long t2 = micros();
  unsigned long t3 = micros();

  while (!stopP1 || !stopP2 || !stopP3) {
    unsigned long now = micros();
//same methods of stepping as before
    if (!stopP1) {
      if (digitalRead(step1.limitPin) == LOW) {
        stopP1 = true; //switch hit
      } else if (now - t1 >= STEP_PERIOD_US) {
        t1 = now;
        stepPulse(step1.stepPin);
      }
    }

    if (!stopP2) {
      if (digitalRead(step2.limitPin) == LOW) {
        stopP2 = true;
      } else if (now - t2 >= STEP_PERIOD_US) {
        t2 = now;
        stepPulse(step2.stepPin);
      }
    }

    if (!stopP3) {
      if (digitalRead(step3.limitPin) == LOW) {
        stopP3 = true;
      } else if (now - t3 >= STEP_PERIOD_US) {
        t3 = now;
        stepPulse(step3.stepPin);
      }
    }
  }

  //moves forward to release limit switches
  stopP1 = stopP2 = stopP3 = true; //start by assuming all need to move forward
  digitalWrite(step1.dirPin, LOW);  //forwards
  digitalWrite(step2.dirPin, HIGH); 
  digitalWrite(step3.dirPin, LOW);

  //reset timers for forward motion
  t1 = t2 = t3 = micros();

  while (stopP1 || stopP2 || stopP3) {
    unsigned long now = micros();

    if (stopP1) {
      if (digitalRead(step1.limitPin) == HIGH) {
        stopP1 = false; //released
      } else if (now - t1 >= STEP_PERIOD_US) {
        t1 = now;
        stepPulse(step1.stepPin);
      }
    }

    if (stopP2) {
      if (digitalRead(step2.limitPin) == HIGH) {
        stopP2 = false;
      } else if (now - t2 >= STEP_PERIOD_US) {
        t2 = now;
        stepPulse(step2.stepPin);
      }
    }

    if (stopP3) {
      if (digitalRead(step3.limitPin) == HIGH) {
        stopP3 = false;
      } else if (now - t3 >= STEP_PERIOD_US) {
        t3 = now;
        stepPulse(step3.stepPin);
      }
    }
  }

  int bumpSteps = 200; //gives switches a bump to ensure limit release

digitalWrite(step1.dirPin, LOW);  //forward
digitalWrite(step2.dirPin, HIGH);
digitalWrite(step3.dirPin, LOW);

for (int i = 0; i < bumpSteps; i++) {
  stepPulse(step1.stepPin);
  stepPulse(step2.stepPin);
  stepPulse(step3.stepPin);
  delayMicroseconds(STEP_PERIOD_US);
}

  lcdMessage("Homing Complete.", "");
  delay(2000);
  lcdMessage("Device ready", "");
  STEPS_PER_SEC = 3000;
  STEP_PERIOD_US = 1000000UL / STEPS_PER_SEC;
}




void enableMotors() {
  digitalWrite(step1.enPin, LOW);
  digitalWrite(step2.enPin, LOW);
  digitalWrite(step3.enPin, LOW);
}

void disableMotors() {
  digitalWrite(step1.enPin, HIGH);
  digitalWrite(step2.enPin, HIGH);
  digitalWrite(step3.enPin, HIGH);
}



