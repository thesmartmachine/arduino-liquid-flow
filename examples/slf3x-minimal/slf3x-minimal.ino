/*
 * Copyright (c) 2019, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 /*
  * This is an early stage prototype; use at your own risk!
  */

#include "sensirion-lf.h"
#include "driver/ledc.h"
#include <math.h>


//GLOBAL VARIABLES
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
const int PWM_PIN0 = 13;
const int PWM_PIN1 = 12; 

const bool DEBUG = true;   //true = serial print operating information

const int CHANNEL0 = 0; 
const int CHANNEL1 = 2;   //Channel 0, 1, share a PWM timer, to make PWM0,PWM1 frequency outputs independent must use separate timer.

const int PWM_RES = 8;    //Channel resolution will determine operating frequency range  
const int PWM_DUTY = 127; //Keep Duty 50%, constant for this application

int READ0 = 0; 
int READ1 = 0;

int FREQ0 = 10;          
int FREQ1 = 10;

int MIN_FREQ = 10; 
int MAX_FREQ = 10000;


//CLASS DEFENITIONS
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
const uint8_t SDA_ZERO = 21;
const uint8_t SCL_ZERO = 22;
const uint8_t I2C_INDEX0 = 0;

SensirionLF SLF3X_I2C_ZERO(
  SensirionLF::SLF3X_SCALE_FACTOR_FLOW,
  SensirionLF::SLF3X_SCALE_FACTOR_TEMP,
  SensirionLF::SLF3X_I2C_ADDRESS,
  I2C_INDEX0,
  SCL_ZERO,
  SDA_ZERO);

const uint8_t SDA_ONE = 26;
const uint8_t SCL_ONE = 25;
const uint8_t I2C_INDEX1 = 1;

SensirionLF SLF3X_I2C_ONE(
  SensirionLF::SLF3X_SCALE_FACTOR_FLOW,
  SensirionLF::SLF3X_SCALE_FACTOR_TEMP,
  SensirionLF::SLF3X_I2C_ADDRESS,
  I2C_INDEX1,
  SCL_ONE,
  SDA_ONE);

//SEQUENCER
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Update rate liquid flow value and High Flow flag @ 1000Hz
#define flow_sense_time_interval 1 //Time in ms
unsigned long flow_sense_time = 0;
bool no_air_in_line_0 = false;

//Update rate temperature value and Air-in-Line flag @ 80Hz
#define air_in_line_interval 13 //Time in ms
unsigned long air_in_line_time = 0;
bool no_air_in_line_1 = false;

//SETUP
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void setup() {
  Serial.begin(115200); // initialize serial communication

  if ((SLF3X_I2C_ZERO.init() != 0) || (SLF3X_I2C_ONE.init() != 0)) {
    Serial.println("SLF3X_0 or SLF3X_1 not initialised. Power Cycling ESP32");
    ESP.restart();
  }

  ledcSetup(CHANNEL0, MIN_FREQ, PWM_RES);
  ledcSetup(CHANNEL1, MIN_FREQ, PWM_RES);

  ledcAttachPin(PWM_PIN0, CHANNEL0);
  ledcAttachPin(PWM_PIN1, CHANNEL1);

  ledcWrite(CHANNEL0, PWM_DUTY);
  ledcWrite(CHANNEL1, PWM_DUTY);
}

//MAIN
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void loop() 
{    
  if (millis() > flow_sense_time + flow_sense_time_interval) {
    READ0 = SLF3X_I2C_ZERO.readSample(); 
    READ1 = SLF3X_I2C_ONE.readSample();   
    
    if (!READ0 && no_air_in_line_0) {
      FREQ0 = round(10000 / 60 * SLF3X_I2C_ZERO.getFlow());
      if (SLF3X_I2C_ZERO.isHighFlowDetected()) FREQ0 = 10000;
    }
    
    if(READ0 != 0) {
      if(DEBUG) Serial.print("Error while reading I2C device 0");
      if(SLF3X_I2C_ZERO.softReset()) ESP.restart();
    }
  
    if (!READ1 && no_air_in_line_1) {
      FREQ1 = round(1000 / 60 * SLF3X_I2C_ONE.getFlow());
      if (SLF3X_I2C_ONE.isHighFlowDetected()) FREQ1 = 10000;
    }
  
    if (READ1 != 0) {
      if(DEBUG) Serial.print("Error while reading I2C device 1");
      if(SLF3X_I2C_ONE.softReset()) ESP.restart(); 
    }
  }
  
  if (millis() > flow_sense_time + air_in_line_interval) {
    if (SLF3X_I2C_ZERO.isAirInLineDetected()) {
      no_air_in_line_0 = false;
      FREQ0 = 2000;  
    }
    if (SLF3X_I2C_ONE.isAirInLineDetected()) {
      no_air_in_line_1 = true; 
      FREQ1 = 2000;  
    }
  }

  ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, constrain(FREQ0, MIN_FREQ, MAX_FREQ));
  ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, constrain(FREQ1, MIN_FREQ, MAX_FREQ));

  if(DEBUG) {Serial.print(FREQ0); Serial.print(", "); Serial.println(FREQ1);}
}

//{
//  
//  if (millis() > flow_sense_time + flow_sense_time_interval)
//  {
//    // Update rate liquid flow value and High Flow flag @ 1000Hz
//    if (no_air_in_line_0) {
//      if (SLF3X_I2C_ZERO.readSample() == 0) FREQ0 = round(10000 / 60 * SLF3X_I2C_ZERO.getFlow());
//      if (SLF3X_I2C_ZERO.isHighFlowDetected()) FREQ0 = 10000; }
//
//    if (no_air_in_line_1) {
//      if (SLF3X_I2C_ONE.readSample() == 0) FREQ1 = round(10000 / 60 * SLF3X_I2C_ONE.getFlow());
//      if (SLF3X_I2C_ONE.isHighFlowDetected()) FREQ1 = 10000; }
//
//    flow_sense_time = millis();
//  }
//
//  if (millis() > air_in_line_time + air_in_line_interval)
//  {
//    // Update rate temperature value and Air-in-Line flag @ 80Hz
//    if (SLF3X_I2C_ZERO.isAirInLineDetected()) {
//      no_air_in_line_0 = false;
//      FREQ0 = 2000;
//    } else {
//      no_air_in_line_0 = true;
//    }
//
//    if (SLF3X_I2C_ONE.isAirInLineDetected()) {
//      no_air_in_line_1 = false;
//      FREQ1 = 2000;
//    } else {
//      no_air_in_line_1 = true;
//    }
//
//    air_in_line_time = millis();
//  }
//
//  // Set PWM outputs
//  ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, constrain(FREQ0, MIN_FREQ, MAX_FREQ));
//  ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, constrain(FREQ1, MIN_FREQ, MAX_FREQ));
//  
//  Serial.print(FREQ0);
//  Serial.print(",");
//  Serial.println(FREQ1);
//}
