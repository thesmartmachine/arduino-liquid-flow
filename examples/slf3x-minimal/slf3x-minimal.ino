#include "sensirion-lf.h"
#include "driver/ledc.h"
#include <math.h>

// delay between measurements
#define MEASURE_DELAY 100

const int PWM_PIN0 = 16;
const int PWM_PIN1 = 17;

const bool DEBUG = false;     //SET Debug mode on = true, debug mode off = false, this will affect whether flow sensor error messages affect PWM output frequency

const int CHANNEL0 = 0;
const int CHANNEL1 = 2;   //Channel 0, 1, share a PWM timer, to make PWM0,PWM1 frequency outputs independent must use separate timer.

const int PWM_RES = 8;    //Channel resolution will determine operating frequency range
const int PWM_DUTY = 127; //Keep Duty 50%, constant for this application

int COUNT0 = 0;           //Timeout counters, power cycle MCU when I2C communication fails X times.
int COUNT1 = 0;

const int TIMEOUT = 10;   //Number of allowable failed communcation attempts on I2C before power cycle will occur.

int FREQ0 = 10;
int FREQ1 = 10;

int MIN_FREQ = 10;
int MAX_FREQ = 10000;
int ERROR_FREQ = 5000;

int READ0 = 0;
int READ1 = 0;

// Create first SLF3X.
const int SDA_ZERO = 21;
const int SCL_ZERO = 22;
const int I2C_INDEX0 = 0;

SensirionLF SLF3X_I2C_ZERO(
  SensirionLF::SLF3X_SCALE_FACTOR_FLOW,
  SensirionLF::SLF3X_SCALE_FACTOR_TEMP,
  SensirionLF::SLF3X_I2C_ADDRESS,
  SDA_ZERO,
  SCL_ZERO,
  I2C_INDEX0);

// Create second SLF3X.
const int SDA_ONE = 26;
const int SCL_ONE = 25;
const int I2C_INDEX1 = 1;

SensirionLF SLF3X_I2C_ONE(
  SensirionLF::SLF3X_SCALE_FACTOR_FLOW,
  SensirionLF::SLF3X_SCALE_FACTOR_TEMP,
  SensirionLF::SLF3X_I2C_ADDRESS,
  SDA_ONE,
  SCL_ONE,
  I2C_INDEX1);

//SEQUENCER
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define flow_sense_time_interval 1 
unsigned long flow_sense_time = 0;
bool no_air_in_line_0 = true;

#define air_in_line_interval 13
unsigned long air_in_line_time = 0;
bool no_air_in_line_1 = true;

//Update rate PWM frequency
#define pwm_freq_interval 200
unsigned long pwm_freq_time = 0;
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup() {
  delay(500);

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

void loop()
{

  if (millis() > flow_sense_time + flow_sense_time_interval)
  {
    READ0 = SLF3X_I2C_ZERO.readSample();
    READ1 = SLF3X_I2C_ONE.readSample();

    if (no_air_in_line_0 && !READ0) {
      FREQ0 = round(1000 / 60 * SLF3X_I2C_ZERO.getFlow());
      if (SLF3X_I2C_ZERO.isHighFlowDetected()) FREQ0 = 10000;
    }

    if (no_air_in_line_1 && !READ1) {
      FREQ1 = round(1000 / 60 * SLF3X_I2C_ONE.getFlow());
      if (SLF3X_I2C_ONE.isHighFlowDetected()) FREQ1 = 10000;
    }

    if (READ0 != 0) {
      if(SLF3X_I2C_ZERO.softReset()) ESP.restart();
    }
    if (READ1 != 0) {
      if(SLF3X_I2C_ONE.softReset()) ESP.restart();
    }

    flow_sense_time = millis();
  }

  if ((millis() > air_in_line_time + air_in_line_interval) && !DEBUG)
  {
    if (SLF3X_I2C_ZERO.isAirInLineDetected()) {
      no_air_in_line_0 = false;
      FREQ0 = 2000;
    } else {
      no_air_in_line_0 = true;
    }

    if (SLF3X_I2C_ONE.isAirInLineDetected()) {
      no_air_in_line_1 = false;
      FREQ1 = 2000;
    } else {
      no_air_in_line_1 = true;
    }

    air_in_line_time = millis();
  }

  if (millis() > pwm_freq_time + pwm_freq_interval) {
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, constrain(FREQ0, MIN_FREQ, MAX_FREQ));
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, constrain(FREQ1, MIN_FREQ, MAX_FREQ));
    pwm_freq_time = millis();
  }

  if (DEBUG) {
    Serial.print(FREQ0);
    Serial.print(",");
    Serial.println(FREQ1);
  }
}
