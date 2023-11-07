#include <Arduino.h>
#include "imxrt.h"
#include "lib/spwm_esc_direct_pid.cpp"
#include "TeensyTimerTool.h"
#include "calibration/voltage-map.cpp"
#include "calibration/ac.cpp"
using namespace TeensyTimerTool;

// Define timers.
PeriodicTimer logging_timer(GPT2), fault_flash_timer(PIT);

// Logging timer interval.
const std::size_t LOGGING_MICROS = 4556; // 40000 // 9111

// LED flash timer interval.
const std::size_t FLASH_MICROS = 100000;

#ifndef DISABLE_LOGGING_CTRS
#define DISABLE_LOGGING_CTRS false
#endif

// Encoder sampler config.
const std::size_t ENCODER_DIVISIONS = 16384;
const std::size_t ENCODER_VALUE_COMPRESSION = 4;

// Encoder pin config.
uint32_t ENC_PIN_CSN = 10;
uint32_t ENC_PIN_MISO = 12;
uint32_t enc_pin_MOSI = 11;
uint32_t enc_pin_SCK = 22;

// Device status LED pin.
const int LED_PIN = 13;

// Kalman config.
double KALMAN_ALPHA = 40000000000.0;
double KALMAN_X_RESOLUTION_ERROR = 4.0;           // 0.00001; // 4.0; // 0.00001;
double KALMAN_PROCESS_NOISE = 0.0000000000000001; // 1000.0; // 10.0; // 0.000000000001;

// spwm pin config.
uint32_t SPWM_PIN_PHASE_A = 1;
uint32_t SPWM_PIN_PHASE_B = 0;
uint32_t SPWM_PIN_PHASE_C = 7;
uint32_t SPWM_PIN_EN = 8;

/* Frequency / pwm write resolution selection.
info from:
https://www.pjrc.com/teensy/td_pulse.html

bits,value     ,freq
15   0 - 32757 4577.64 Hz
14   0 - 16383 9155.27 Hz
13   0 - 8191  18310.55 Hz
12   0 - 4095  36621.09 Hz
11   0 - 2047  73242.19 Hz
10   0 - 1023  146484.38 Hz
9    0 - 511   292968.75 Hz
*/
const std::size_t PWM_WRITE_RESOLUTION = 11;
uint32_t SPWM_PIN_PWM_FREQUENCY = 70000;

// Define encoder pin config struct.
kaepek::DigitalEncoderPinsSPI ENC_PINS = kaepek::DigitalEncoderPinsSPI();

// Define the encoder.
kaepek::DigitalRotaryEncoderSPI ENC;

// kalman config struct.
kaepek::KalmanConfig KALMAN_CONFIG = kaepek::KalmanConfig();

// SPWM pin config struct.
kaepek::SPWML6234PinConfig SPWM_PIN_CONFIG = kaepek::SPWML6234PinConfig();

// Define the PID struct.
kaepek::PIDConfig PID_CONFIG = kaepek::PIDConfig();

// Define the encoder ESC.
kaepek::PidEscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_VALUE_COMPRESSION, PWM_WRITE_RESOLUTION> ESC;

// Define the ESC duty cap
double DUTY_CAP = 0.3;

// Define routine to turn status led pin on and off.
bool LED_STATUS_ON = false;
void fault_flash()
{
  if (LED_STATUS_ON == true)
  {
    digitalWrite(LED_PIN, LOW);
    LED_STATUS_ON = false;
  }
  else
  {
    digitalWrite(LED_PIN, HIGH);
    LED_STATUS_ON = true;
  }
}

// Define a routine to log out ESC data.
bool logging_timer_started = false;
void log_data()
{
  // If we started and did not have a fault then log.
  if (ESC.get_fault_status() == false && ESC.get_started_status() == true)
  {
    ESC.log();
  }
  else // If we have not started or have a fault then stop logging.
  {
    logging_timer.stop();
    logging_timer_started = false;
    if (ESC.get_fault_status() == true)
    {
      // Enable fault state for status led.
      fault_flash_timer.begin(fault_flash, FLASH_MICROS);
    }
    else if (ESC.get_started_status() == false)
    {
      // Turn off status led.
      digitalWrite(LED_PIN, LOW);
      fault_flash_timer.stop();
    }
  }
}

void setup()
{
  // Setup status LED pin.
  pinMode(LED_PIN, OUTPUT);

  // PID_CONFIG
  PID_CONFIG.proportional = 0.0;
  PID_CONFIG.integral = 0.0;
  PID_CONFIG.differential = 0.0;
  PID_CONFIG.power_law_set_point_divisor_cw = 0.0;
  PID_CONFIG.power_law_root_cw = 0.0;
  PID_CONFIG.power_law_set_point_divisor_ccw = 0.0;
  PID_CONFIG.power_law_root_ccw = 0.0;
  PID_CONFIG.linear_set_point_coefficient_cw = 0.0;
  PID_CONFIG.linear_set_point_coefficient_ccw = 0.0;
  PID_CONFIG.linear_bias_cw = 0.0;
  PID_CONFIG.linear_bias_ccw = 0.0;

  // KALMAN_CONFIG.
  KALMAN_CONFIG.alpha = KALMAN_ALPHA;
  KALMAN_CONFIG.x_resolution_error = KALMAN_X_RESOLUTION_ERROR;
  KALMAN_CONFIG.process_noise = KALMAN_PROCESS_NOISE;

  // SPWM_PIN_CONFIG.
  SPWM_PIN_CONFIG.phase_a = SPWM_PIN_PHASE_A;
  SPWM_PIN_CONFIG.phase_b = SPWM_PIN_PHASE_B;
  SPWM_PIN_CONFIG.phase_c = SPWM_PIN_PHASE_C;
  SPWM_PIN_CONFIG.en = SPWM_PIN_EN;
  SPWM_PIN_CONFIG.frequency = SPWM_PIN_PWM_FREQUENCY;

  // Setup the encoder pin configuration.
  ENC_PINS.csn = ENC_PIN_CSN;
  ENC_PINS.miso = ENC_PIN_MISO;
  ENC_PINS.mosi = enc_pin_MOSI;
  ENC_PINS.sck = enc_pin_SCK;

  // Initalise the encoder with giving it the pin configuration.
  ENC = kaepek::DigitalRotaryEncoderSPI(ENC_PINS);

  // Initalise the encoder ESC.
  ESC = kaepek::PidEscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_VALUE_COMPRESSION, PWM_WRITE_RESOLUTION>(DUTY_CAP, ENC, 3.8, SPWM_PIN_CONFIG, KALMAN_CONFIG, PID_CONFIG, VOLTAGE_MAP, AC_MAP); // 3us (micro) sample period 2.8 2.6

  // Allow skipping ahead a maximum value of 4.0, in terms of the read encoder value measurement, before a skip is detected.
  ESC.set_skip_tolerance(8.0);
  // Only allow skipping ahead twice before faulting.
  ESC.set_skip_threshold(2);

  // To disable direction enforcement.
  ESC.set_direction_enforcement(false);

  // Run setup procedure of the ESC. Note this will invoke the encoder's setup method and therefore it is unnecessary to do it explicitly on the encoder instance.
  ESC.setup();
}

void loop()
{
  // Perform the ESC loop tick function.
  ESC.loop();

  // If we have started but not started the logging timer then start it.
  if (logging_timer_started == false && ESC.get_started_status() == true)
  {
    // Start logging.
    logging_timer.begin(log_data, LOGGING_MICROS);
    logging_timer_started = true;
    // Turn on status LED pin.
    fault_flash_timer.stop();
    digitalWrite(LED_PIN, HIGH);
  }

  if (ESC.get_started_status() == false && ESC.get_fault_status() == false)
  {
    // Turn off status LED pin.
    fault_flash_timer.stop();
    digitalWrite(LED_PIN, LOW);
  }
}
