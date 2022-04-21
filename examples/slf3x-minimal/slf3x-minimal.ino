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

// delay between measurements
#define MEASURE_DELAY 100

const int PWM_PIN0 = 13;
const int PWM_PIN1 = 12; 

const bool DEBUG = true;     //SET Debug mode on = true, debug mode off = false, this will affect whether flow sensor error messages affect PWM output frequency

const int BOOT_FREQ = 10;

const int CHANNEL0 = 0; 
const int CHANNEL1 = 2;   //Channel 0, 1, share a PWM timer, to make PWM0,PWM1 frequency outputs independent must use separate timer.

const int PWM_RES = 8;    //Channel resolution will determine operating frequency range  
const int PWM_DUTY = 127; //Keep Duty 50%, constant for this application

int COUNT0 = 0;           //Timeout counters, power cycle MCU when I2C communication fails X times.
int COUNT1 = 0;

const int TIMEOUT = 10;   //Number of allowable failed communcation attempts on I2C before power cycle will occur. 

int FREQ0 = 0;          
int FREQ1 = 0;

// Create first SLF3X.
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

// Create second SLF3X.
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

void setup() {
  Serial.begin(115200); // initialize serial communication

  if (SLF3X_I2C_ZERO.init() != 0) {
    Serial.println("Error during SLF3X_0 init. Power Cycling ESP32");
    ESP.restart();
  }

  if (SLF3X_I2C_ONE.init() != 0) {
    Serial.println("Error during SLF3X_1 init. Stopping application.");
    ESP.restart();
  }

  ledcSetup(CHANNEL0, BOOT_FREQ, PWM_RES);
  ledcSetup(CHANNEL1, BOOT_FREQ, PWM_RES);

  ledcAttachPin(PWM_PIN0, CHANNEL0);
  ledcAttachPin(PWM_PIN1, CHANNEL1);

  ledcWrite(CHANNEL0, PWM_DUTY);
  ledcWrite(CHANNEL1, PWM_DUTY);
}

void loop() {
  int I2C_RET0 = SLF3X_I2C_ZERO.readSample();
  if (I2C_RET0 == 0) {
    COUNT0 = 0;

    FREQ0 = round(10000/60 * SLF3X_I2C_ZERO.getFlow());

    if (FREQ0 < 5) {
      FREQ0 = 5;
      Serial.print("FLOW0 LOW");
    }

    if (!DEBUG) {
      if (SLF3X_I2C_ZERO.isAirInLineDetected()){
        FREQ0 = 2000;
        Serial.print(" [ ERROR : AIR IN LINE 0 ] ");
      }
      if (SLF3X_I2C_ZERO.isHighFlowDetected()) {
        FREQ0 = 10000;
        Serial.print(" [ ERROR : HIGH FLOW IN LINE 0 ] ");
      }
    }

    
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, FREQ0);


    Serial.print("Flow 0: ");
    Serial.print(SLF3X_I2C_ZERO.getFlow(), 2);
    Serial.print(" ml/min");

    Serial.print(" | Temp 0: ");
    Serial.print(SLF3X_I2C_ZERO.getTemp(), 1);
    Serial.print(" deg C  |  FREQ0: ");
    Serial.print(FREQ0);
  } else {
    Serial.print("Error in SLF3X_0.readSample(): ");
    COUNT0++; 
    if (COUNT0 > 9) {
      ESP.restart();
    }
  }

  int I2C_RET1 = SLF3X_I2C_ONE.readSample();
  if (I2C_RET1 == 0) {
    COUNT1 = 0;

    FREQ1 = round(10000/60 * SLF3X_I2C_ONE.getFlow());

    if (FREQ1 < 5) {
      FREQ1 = 5;  
      Serial.print("FLOW1 LOW");
    } 
    
    if (!DEBUG) {
      if (SLF3X_I2C_ONE.isAirInLineDetected()){
        FREQ1 = 2000;
        Serial.print(" [ ERROR : AIR IN LINE 1 ] ");
      }
      if (SLF3X_I2C_ONE.isHighFlowDetected()) {
        FREQ1 = 10000;
        Serial.print(" [ ERROR : HIGH FLOW IN LINE 1 ] ");
      }
    }
    
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, FREQ1);

    Serial.print("   Flow 1: ");
    Serial.print(SLF3X_I2C_ONE.getFlow(), 2);
    Serial.print(" ml/min");

    Serial.print(" | Temp 1: ");
    Serial.print(SLF3X_I2C_ONE.getTemp(), 1);
    Serial.print(" deg C  |  FREQ 1: ");
    Serial.println(FREQ1);
  } else {
    Serial.print("Error in SLF3X_1.readSample(): ");
    Serial.println();
    COUNT1++;
    if (COUNT1 > 9) {
      ESP.restart();
    }
  }

  delay(MEASURE_DELAY); // delay between reads
}
