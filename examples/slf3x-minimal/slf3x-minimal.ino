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

// delay between measurements
#define MEASURE_DELAY 100

// Create first SLF3X.
const uint8_t ZERO_I2C_BUS = 0;
const uint8_t SDA_ZERO = 21;
const uint8_t SCL_ZERO = 22;
SensirionLF SLF3X_I2C_ZERO(
  SensirionLF::SLF3X_SCALE_FACTOR_FLOW,
  SensirionLF::SLF3X_SCALE_FACTOR_TEMP,
  ZERO_I2C_BUS,
  SensirionLF::SLF3X_I2C_ADDRESS,
  SCL_ZERO,
  SDA_ZERO);

// Create second SLF3X.
const uint8_t ONE_I2C_BUS = 0;
const uint8_t SDA_ONE = 26;
const uint8_t SCL_ONE = 25;
SensirionLF SLF3X_I2C_ONE(
  SensirionLF::SLF3X_SCALE_FACTOR_FLOW,
  SensirionLF::SLF3X_SCALE_FACTOR_TEMP,
  ONE_I2C_BUS,
  SensirionLF::SLF3X_I2C_ADDRESS,
  SCL_ONE,
  SDA_ONE);

void setup() {
  Serial.begin(115200); // initialize serial communication

  if (SLF3X_I2C_ZERO.init() != 0) {
    Serial.println("Error during SLF3X_0 init. Stopping application.");
    while (1) { delay(1000); } // loop forever
  }

  if (SLF3X_I2C_ONE.init() != 0) {
    Serial.println("Error during SLF3X_1 init. Stopping application.");
    while (1) { delay(1000); } // loop forever
  }
}

void loop() {
  int ret = SLF3X_I2C_ZERO.readSample();
  if (ret == 0) {
    Serial.print("Flow: ");
    Serial.print(SLF3X_I2C_ZERO.getFlow(), 2);
    Serial.print(" ml/min");

    Serial.print(" | Temp: ");
    Serial.print(SLF3X_I2C_ZERO.getTemp(), 1);
    Serial.print(" deg C\n");
  } else {
    Serial.print("Error in SLF3X_0.readSample(): ");
    Serial.println(ret);
  }

  int ret = SLF3X_I2C_ONE.readSample();
  if (ret == 0) {
    Serial.print("Flow: ");
    Serial.print(SLF3X_I2C_ONE.getFlow(), 2);
    Serial.print(" ml/min");

    Serial.print(" | Temp: ");
    Serial.print(SLF3X_I2C_ONE.getTemp(), 1);
    Serial.print(" deg C\n");
  } else {
    Serial.print("Error in SLF3X_1.readSample(): ");
    Serial.println(ret);
  }

  delay(MEASURE_DELAY); // delay between reads
}
