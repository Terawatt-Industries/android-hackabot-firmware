/*
Copyright 2013 Longevity Software LLC, d.b.a. Terawatt Industries

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

Author:  Free Beachler

TSL230 code is based on http://dronecolony.com/2008/11/13/arduino-and-the-taos-tsl230r-light-sensor-getting-started/
*/

//#define DEBUG_MODE
#include "RunningAverage.h"

#define _VERSION 4
// WS2801 pins:
// pin 5 = CKI
// pin 6 = SDI
#define CKI 4
#define SDI 5
#define NUMBER_OF_LEDS  1  // number of LEDs attached to ws2801
unsigned long pallette[16] = {
  0xFF9A00, 
  0xBB00BB, 
  0x800000, 
  0x11FF00, 
  0x0000FF, 
  0xFF1100, 
  0x550022, 
  0x110099, 
  0xFF1100, 
  0xEF2203, 
  0xFF1100, 
  0xD900FF, 
  0xFF4420, 
  0xAA00FF, 
  0x226498, 
  0x698410
  };
unsigned long ledColor[NUMBER_OF_LEDS] = {pallette[4]};
byte ledState[NUMBER_OF_LEDS] = {0};
byte ledStateIndex;

#define SERVO_PIN 9
#define MAX_PULSE_WIDTH = 2500;
#define MIN_PULSE_WIDTH = 540;

// variables used to track time
unsigned long curTIME = millis();
unsigned long preTIMEL2FChecked = curTIME;

#include <Servo.h>
Servo servo1;  // create servo object to control a servo 
int servoPos = 0;

#define SCENE_DELAY 200;
//int motionScenesHeadJar[9] = {540, 2200, 650, 2500, 540, 2100, 740, 2500, 540};
int motionScenesHeadJar[9] = {540, -1, -1, -1, 200, -1, -1, -1, 540};  // -1 means random
byte motionSceneIndex = 0;
unsigned long ttlNextScene = millis();
boolean scenesExecuting = false;

// light to freq (L2F) = TSL235 IC
#define L2FIRQPIN 2      // interrupt pin on arduino
#define READ_TM 50      // 1000ms = 1s
unsigned long l2fPulseCnt = 0;
// we'll need to access the amount
// of time passed
unsigned int tm_diff = 0;
// need to measure what to divide freq by
// 1x sensitivity = 10,
// 10x sens       = 100,
// 100x sens      = 1000
int calc_sensitivity = 10;
// set our frequency multiplier to a default of 1
int freq_mult = 1;
unsigned int currLum = 0;
unsigned int lastLum = 0;
int trig_cnt = 0;
unsigned long trig_seq_start_time = 0;
int action_val = 0;
boolean sleeping = false;

// running avg for light meter sampling
boolean reset_samples = false;
#define RA_SAMPLE_SIZE 8
RunningAverage l2f_ra(RA_SAMPLE_SIZE);
int ra_sample_cnt = 0;

void setup()
{
  // setup LEDs
  pinMode(SDI, OUTPUT);
  pinMode(CKI, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  ledStateIndex = 0;
  // setup servo
  servo1.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  servo1.write(0);
  delay(50);
  servo1.write(180);
  delay(50);
  servo1.detach();
  // setup light 2 frequency converter
  pinMode(L2FIRQPIN, INPUT);
  attachInterrupt(0, add_pulse, RISING);
  // re-set the ms counter
  tm_diff = 0;
  // get our current frequency reading
  unsigned long frequency = get_tsl_freq();
  // calculate radiant energy
  float uw_cm2 = calc_uwatt_cm2(frequency);
  // calculate illuminance
  float lux = calc_lux_single(uw_cm2, 0.175);
  l2f_ra.clear();
  l2f_ra.addValue(frequency);
  ra_sample_cnt++;
#ifdef DEBUG_MODE
  Serial.begin(9600);
  Serial.print("setup done...");
  Serial.print("\n");
#endif
}

void loop()
{
  refreshLEDs();
  // check the value of the light sensor every READ_TM ms
  // calculate how much time has passed
  preTIMEL2FChecked = curTIME;
  curTIME   = millis();
  if (curTIME > preTIMEL2FChecked) {
    tm_diff += curTIME - preTIMEL2FChecked;
  } else if (curTIME < preTIMEL2FChecked) {
    // handle overflow and rollover (Arduino 011)
    tm_diff += (curTIME + (34359737 - preTIMEL2FChecked));
  }
  if (reset_samples) {
#ifdef DEBUG_MODE
  Serial.print("** sampels reset");
  Serial.print("\n");
#endif
    reset_samples = false;
    get_tsl_freq();
    curTIME   = millis();
    preTIMEL2FChecked = curTIME;
    l2f_ra.clear();
    ra_sample_cnt = 0;
    tm_diff = 0;
  }
  if (!scenesExecuting) {
    // if enough time has passed to do a new reading...
    if (tm_diff >= READ_TM) {
      // re-set the ms counter
      tm_diff = 0;
      // get our current frequency reading
      unsigned long frequency = get_tsl_freq();
      l2f_ra.addValue(frequency);
      ra_sample_cnt++;
      if (ra_sample_cnt == RA_SAMPLE_SIZE) {
        currLum = l2f_ra.getAverage();  //getAverage() is not optimized, but we only call when needed
        l2f_ra.clear();
        ra_sample_cnt = 0;
#ifdef DEBUG_MODE
  Serial.print("currLum = ");
  Serial.print(currLum, DEC);
  Serial.print("\n");
#endif
      }
      // we don't do anything with these calcs
      // calculate radiant energy
      float uw_cm2 = calc_uwatt_cm2(frequency);
      // calculate illuminance
      float lux = calc_lux_single(uw_cm2, 0.175);
    }
    int delta = abs((int) lastLum - (int) currLum);
    if (300 < delta) {
      lastLum = currLum;
      if (trig_cnt == 0) {
        trig_seq_start_time = millis();
      }
      // increment trigger
      trig_cnt++;
    }
    // analyze triggers during pause
    if (trig_cnt > 0 && millis() - trig_seq_start_time > 1500) {
#ifdef DEBUG_MODE
  Serial.print("analyze trigger...");
  Serial.print("  trig_cnt = ");
  Serial.print(trig_cnt, DEC);
  Serial.print("\n");
#endif
      if (trig_cnt > 2) {
        action_val = 1;  // dance!
      } else if (trig_cnt == 2) {
        action_val = 2;  // toggle state
      }
      trig_cnt = 0;
    }
    if (action_val > 0) {
#ifdef DEBUG_MODE
  Serial.print("taking action...");
  Serial.print("\n");
  Serial.print("delta = ");
  Serial.print(delta, DEC);
  Serial.print("currLum = ");
  Serial.print(currLum, DEC);
  Serial.print("lastLum = ");
  Serial.print(lastLum, DEC);
  Serial.print("action_val = ");
  Serial.print(action_val, DEC);
  Serial.print("\n");
#endif
      if (action_val == 1) {
      // play scenes
      motionSceneIndex = 0;
      scenesExecuting = true;
      servo1.attach(SERVO_PIN);
      ledState[0] = 1;
      ledState[1] = 1;
      ledColor[0] = pallette[random(0, 15)];
      ledColor[1] = pallette[random(0, 15)];
      int servoUs = motionScenesHeadJar[motionSceneIndex];
      if (-1 == servoUs) {
        // -1 means random
        servoUs = random(MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
      }
      servo1.write(servoUs);
      ttlNextScene = millis() + (servoUs / 1000) + 50 + SCENE_DELAY;
      }
      if (action_val == 2) {
        if (sleeping) {
          sleeping = false;
          ledColor[0] = 0x00a5d5;
        } else {
          sleeping = true;
          ledColor[0] = 0xDA4040;
        }
        action_val = 0;
      }
    }
  } else {
    // playing scenes
    if (millis() > ttlNextScene) {
      // advance scene
      if (9 == motionSceneIndex) {
        // end of scenes
        scenesExecuting = false;
        motionSceneIndex = 0;
        ledState[0] = 0;
        ledState[1] = 0;
        servo1.detach();
        action_val = 0;
        // reset interrupt counter states
        reset_samples = true;
      } else {
        motionSceneIndex++;
        ledColor[0] = pallette[random(0, 15)];
        ledColor[1] = pallette[random(0, 15)];
        int servoUs = motionScenesHeadJar[motionSceneIndex];
        if (-1 == servoUs) {
          // -1 means random
          servoUs = random(MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
        }
        servo1.write(servoUs);
        ttlNextScene = millis() + (servoUs / 1000) + 50 + SCENE_DELAY;
      }
    }
  }
}

void refreshLEDs() {
  //Each LED requires 24 bits of data
  //MSB: R7, R6, R5..., G7, G6..., B7, B6... B0 
  //Once the 24 bits have been delivered, the IC immediately relays these bits to its neighbor
  //Pulling the clock low for 500us or more causes the IC to post the data.
  for(int ledNum = 0 ; ledNum < NUMBER_OF_LEDS ; ledNum++) {
    long this_led_color = ledColor[ledNum]; //24 bits of color data

    for(byte color_bit = 23 ; color_bit != 255 ; color_bit--) {
      //Feed color bit 23 first (red data MSB)
      digitalWrite(CKI, LOW); //Only change data when clock is low
      long mask = 1L << color_bit;
      //The 1'L' forces the 1 to start as a 32 bit number, otherwise it defaults to 16-bit.
      if(this_led_color & mask) {
        digitalWrite(SDI, HIGH);
      }
      else {
        digitalWrite(SDI, LOW);
      }
      digitalWrite(CKI, HIGH); //Data is latched when clock goes high
    }
  }
  //Pull clock low to put strip into reset/post mode
  digitalWrite(CKI, LOW);
  delayMicroseconds(500); //Wait for 500us to go into reset
}

void add_pulse() {
  // increase pulse count
  l2fPulseCnt++;
  return;
}

unsigned long get_tsl_freq() {
  // we have to scale out the frequency --
  // Scaling on the TSL230R requires us to multiply by a factor
  // to get actual frequency
#ifdef DEBUG_MODE
Serial.print(l2fPulseCnt, DEC);
Serial.print(" = pulse_count");
Serial.print("\n");
#endif
  unsigned long freq = l2fPulseCnt;
  // reset the pulse counter
  l2fPulseCnt = 0;
  return(freq);
}

float calc_uwatt_cm2(unsigned long freq) {
  // get uW observed - assume 640nm wavelength
  // calc_sensitivity is our divide-by to map to a given signal strength
  // for a given sensitivity (each level of greater sensitivity reduces the signal
  // (uW) by a factor of 10)
  float uw_cm2 = (float) freq / (float) calc_sensitivity;
    // extrapolate into entire cm2 area
  uw_cm2 *= ((float) 1 / (float) 0.0136);
  return uw_cm2;
}

/* calculate lux (lm/m^2), using standard formula:
 * Xv = Xl * V(l) * Km
 * Xl is W/m^2 (calculate actual receied uW/cm^2, extrapolate from sensor size (0.0136cm^2)
 * to whole cm size, then convert uW to W)
 * V(l) = efficiency function (provided via argument)
 * Km = constant, lm/W @ 555nm = 683 (555nm has efficiency function of nearly 1.0)

 * Only a single wavelength is calculated - you'd better make sure that your
 * source is of a single wavelength...  Otherwise, you should be using
 * calc_lux_gauss() for multiple wavelengths
*/
float calc_lux_single(float uw_cm2, float efficiency) {
  // convert to w_m2
  float w_m2 = (uw_cm2 / (float) 1000000) * (float) 100;
  // calculate lux
  float lux  = w_m2 * efficiency * (float) 683;
  return(lux);
}

