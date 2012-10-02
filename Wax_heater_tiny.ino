/* Wax_heater_tiny  Luke Miller Sep 2012
 An attempt to run my wax heater on an ATtiny84
 */
#include "TinyWireM.h"
#include "Tiny_LEDBackpack.h"
#include <PID_v1.h> //from https://github.com/br3ttb/Arduino-PID-Library/
Tiny_7segment sevenseg = Tiny_7segment();
//   ATtiny84 physical pin9 --> SCL pin on LED Backpack
//   ATtiny84 physical pin7 --> SDA pin on LED Backpack
#define i2c_addr 0x70 // stock address for Adafruit 7-segment LED backpack
#define button1 2 // pin 2 (physical pin 11), used as digital in for button input
#define ssrPin 8 // digital pin 8 (physical pin 5), used as digital out for SSR
#define led 10 // digital pin 10 (physical pin 2), used as LED indicator
#define TMP36 1 // analog input 1, for reading TMP36
#define aref_voltage 5.0 // analog reference voltage
#define timeLimit 3600000 // value in milliseconds, 3600000 = 1 hour
//#define lowerTempOffset 2 // Temperature difference that will trigger SSR on
#define lowerChoiceTemp 36 // Lowest available temperature option
#define upperChoiceTemp 76 // Highest available temperature option

volatile int tempLimit = 60; // initial temperature target
int rawAnalog; // value to hold raw analog value
float currTemp; // value to hold converted temperature value

//*****************************************
// PID setup
//Define Variables we'll be connecting to

double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,200,100,10, REVERSE);

int WindowSize = 5000;
unsigned long windowStartTime; 
//*****************************************


void setup() {
  analogReference(DEFAULT); //5V reference
  pinMode(button1, INPUT); //button for selecting options
  digitalWrite(button1, HIGH); // set internal pullup resistor 
  pinMode(ssrPin, OUTPUT); //pin to toggle solid state relay (SSR)
  digitalWrite(ssrPin, LOW); //turn off power to SSR initially

  pinMode(led, OUTPUT);
  digitalWrite(led,HIGH);
  delay(1500);
  digitalWrite(led,LOW);
  delay(300); 
  sevenseg.begin(i2c_addr); // initialize HT16K33 controller
  sevenseg.clear(); // clear all digits on display
  sevenseg.writeDisplay();
  tempLimit = tempSetFunc(); // call tempSetFunc
  //-------------------------
  // PID initialization
  Setpoint = tempLimit;
  windowStartTime = millis();
  myPID.SetOutputLimits(0,WindowSize);
  // Turn the PID on
  myPID.SetMode(AUTOMATIC);
  //-------------------------
}

// Main loop
void loop(){
  unsigned long loopTime = millis();  // timer for display updates
  unsigned long loopTime2 = millis(); // timer for PID loop
  while ( millis() < timeLimit ) { // stop running when timeLimit is passed
    
    // Try to update PID routine every 200ms
    if (millis() - loopTime2 > 200) {
      loopTime2 = millis(); // update timer
      Input = (double) getTempFunc();
      myPID.Compute();
      /************************************************
       * Turn the output pin on/off based on pid output
       ************************************************/
      if( (millis() - windowStartTime) > WindowSize)
      { //time to shift the Relay Window
        windowStartTime += WindowSize;
      }
      if( Output < (millis() - windowStartTime) ) {
        digitalWrite(ssrPin,HIGH);
        digitalWrite(led,HIGH);
      }
      else {
        digitalWrite(ssrPin,LOW);
        digitalWrite(led,LOW);
      } 
    }
    // Only update temperature display every second
    if (millis() - loopTime > 1000) {
      loopTime = millis(); // update timer
      currTemp = getTempFunc();
      sevenseg.print(currTemp,1);
      sevenseg.writeDisplay();
    }
     
    // Check button1 to see if user is providing input
    // to change setpoint 
    if (digitalRead(button1) == LOW) {
      delay(5); // crude debounce
      if (digitalRead(button1) == LOW) { // if button is still pressed
        sevenseg.clear();    
        sevenseg.writeDisplay();
        if (Setpoint >= lowerChoiceTemp & Setpoint < upperChoiceTemp) {
          Setpoint = Setpoint++;    // raise Setpoint 1 degree
      } else if (Setpoint >= upperChoiceTemp) {
          Setpoint = lowerChoiceTemp; // return to lower limit
      }
        sevenseg.print(Setpoint, 0); // show new Setpoint value
        sevenseg.writeDisplay();
        delay(300);
      }
    }
      
    // Non-PID version, in case the PID isn't useful
//       if (millis() - loopTime > 1000) {
//         loopTime = millis(); // update loopTime
//         currTemp = getTempFunc(); // get current temperature 
//          
//          // Compare temperature to setpoint, turn solid state relay
//          // on or off as necessary.
//          if (currTemp < tempLimit - lowerTempOffset) {
//            // If currTemp is less than the lower temperature limit, turn on heater
//           digitalWrite(ssrPin, HIGH);
//           digitalWrite(led, HIGH); 
//          }
//          else if (currTemp > tempLimit) {
//           digitalWrite(ssrPin, LOW);
//          digitalWrite(led, LOW);
//          }
//       }
    //  
  } // end of while loop

  // When the millis counter exceeds the timeLimit,
  // the SSR should be shut off to stop any further
  // heating. The user can reset the chip to 
  // continue using the apparatus. 
  digitalWrite(ssrPin, LOW); // turn off solid state relay
  digitalWrite(led, LOW);
  // Notify user that heat is switched off
  sevenseg.writeDigitRaw(0,63);  // 'O'
  sevenseg.writeDigitRaw(1,113);   // 'F'
  sevenseg.writeDigitRaw(3,113);   // 'F'
  sevenseg.writeDigitRaw(4,0);    // blank
  sevenseg.writeDisplay();
}

//*********************************************************************************
// getTempFunc function. This function takes an analog reading, returns the 
// temperature value, and displays the current temperature on the seven segment 
// display.
float getTempFunc() {
  rawAnalog = analogRead(TMP36);
  rawAnalog = 0;
  delay(10);
  // Take three temperature readings
  for (int i = 0; i < 3; i++) {
    rawAnalog = rawAnalog + analogRead(TMP36); 
    delay(10);
  }
  // Convert to voltage using reference voltage value,
  // and also convert to degrees celsius by subtracting off
  // the 500mV offset and multiplying resulting volts by 100.
  float Temp = (((rawAnalog / 3.0) * aref_voltage / 1024.0) - 0.5) * 100;  
//  sevenseg.print(Temp,1); // only show 1 digit after decimal point
//  sevenseg.writeDisplay(); // push data to display
  return Temp;
}


//**********************************************************************************
// tempSetFunc subroutine. This lets the user choose the temperature limit for 
// the heating element. This uses input from button1 and returns a value
// 'tempLimit' to the setup loop.
int tempSetFunc() { //lowTempFunc will return an integer when called
  int buttonValue1;
  int buttonValue2;
  int buttonState;

  unsigned long startTime = millis(); //get starting time for this loop
  sevenseg.clear();
  sevenseg.writeDisplay();
  sevenseg.writeDigitRaw(0,109);  // 'S'
  sevenseg.writeDigitRaw(1,121);  // 'e'
  sevenseg.writeDigitRaw(3,112);   // 't', alternately 70
  sevenseg.writeDisplay();

  delay(1500);
  sevenseg.print(tempLimit); // print starting temperature
  sevenseg.writeDisplay();
  sevenseg.blinkRate(1); // turn on blinking function

  buttonState = digitalRead(button1); //get current state of button (should be HIGH)

  while (millis() <= startTime + 3500) //while current millis is less than 3sec from startTime
  {
    buttonValue1 = digitalRead(button1);
    delay(10); //perform a crude debounce by checking button twice over 10ms
    buttonValue2 = digitalRead(button1);
    if (buttonValue1 == buttonValue2) {
      if (buttonValue1 != buttonState) { //make sure button state has changed
        if (buttonValue1 == LOW) { //if button is pressed
          if (tempLimit >= lowerChoiceTemp & tempLimit < upperChoiceTemp) {
            tempLimit = tempLimit + 2;
          } 
          else if(tempLimit>= upperChoiceTemp) {
            tempLimit = lowerChoiceTemp;
          }
          sevenseg.print(tempLimit); 
          sevenseg.writeDisplay();
          startTime = millis(); //update startTime to give user more time 
          //to choose another value
        }
      }
      buttonState = buttonValue1; //update buttonState so that only changes
      //in button status are registered
    }
  }
  sevenseg.blinkRate(0);
  sevenseg.clear();
  sevenseg.writeDisplay();
  sevenseg.writeDigitRaw(0,80);  // 'r'
  sevenseg.writeDigitRaw(1,28);  // 'u'
  sevenseg.writeDigitRaw(3,84);  // 'n'
  sevenseg.writeDisplay();
  delay(1500);
  sevenseg.clear();
  sevenseg.writeDisplay();  


  return tempLimit;
}

