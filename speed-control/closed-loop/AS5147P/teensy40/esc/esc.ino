#include <Arduino.h>
#include "imxrt.h"
#include "lib/spwm_esc.cpp"

// Encoder sampler config
const std::size_t ENCODER_DIVISIONS = 16384;
const std::size_t ENCODER_VALUE_COMPRESSION = 4;

// encoder pin config

uint32_t enc_pin_csn = 10;
uint32_t enc_pin_miso = 12;
uint32_t enc_pin_mosi = 11;
uint32_t enc_pin_sck = 22;

// Motor constants
double motor_config_cw_zero_displacement_deg = -1.86;
double motor_config_cw_phase_displacement_deg = 240.01;
double motor_config_ccw_zero_displacement_deg = 15.31;
double motor_config_ccw_phase_displacement_deg = 119.99;
uint32_t motor_config_number_of_poles = 14;

// Kalman config

double kalman_alpha = 50000.0;
double kalman_angular_resolution_error = 40.0;
double kalman_process_noise = 0.000000000001;

// spwm pin config
uint32_t spwm_pin_phase_a = 1;
uint32_t spwm_pin_phase_b = 0;
uint32_t spwm_pin_phase_c = 7;
uint32_t spwm_pin_en = 8;
/* Frequency / pwm write resolution 
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
uint32_t spwm_pin_pwm_frequency = 36000;

// Define encoder pin config struct.
kaepek::DigitalEncoderPinsSPI enc_pins = kaepek::DigitalEncoderPinsSPI();
// Define the encoder.
kaepek::DigitalRotaryEncoderSPI enc;

// kalman config struct
kaepek::KalmanConfig kalman_config = kaepek::KalmanConfig();

// motor calibration struct
kaepek::SPWMMotorConfig motor_calibration_config = kaepek::SPWMMotorConfig();

// spwm pin config struct
kaepek::SPWMPinConfig spwm_pin_config = kaepek::SPWMPinConfig();

// Define the encoder esc.
kaepek::EscTeensy40AS5147P<ENCODER_DIVISIONS, ENCODER_VALUE_COMPRESSION, PWM_WRITE_RESOLUTION> esc;

void setup()
{
  // kalman_config
  kalman_config.alpha = kalman_alpha;
  kalman_config.angular_resolution_error = kalman_angular_resolution_error;
  kalman_config.process_noise = kalman_process_noise;

  // motor_calibration_config
  motor_calibration_config.cw_zero_displacement_deg = motor_config_cw_zero_displacement_deg;
  motor_calibration_config.cw_phase_displacement_deg = motor_config_cw_phase_displacement_deg;
  motor_calibration_config.ccw_zero_displacement_deg = motor_config_ccw_zero_displacement_deg;
  motor_calibration_config.ccw_phase_displacement_deg = motor_config_ccw_phase_displacement_deg;
  motor_calibration_config.number_of_poles = motor_config_number_of_poles;

  // spwm_pin_config
  spwm_pin_config.phase_a = spwm_pin_phase_a;
  spwm_pin_config.phase_b = spwm_pin_phase_b;
  spwm_pin_config.phase_c = spwm_pin_phase_c;
  spwm_pin_config.en = spwm_pin_en;
  spwm_pin_config.frequency = spwm_pin_pwm_frequency;

  // Setup the encoder pin configuration.
  enc_pins.csn = enc_pin_csn;
  enc_pins.miso = enc_pin_miso;
  enc_pins.mosi = enc_pin_mosi;
  enc_pins.sck = enc_pin_sck;

  // Initalise the encoder with giving it the pin configuration.
  enc = kaepek::DigitalRotaryEncoderSPI(enc_pins);

  // Initalise the encoder esc.
  esc = kaepek::EscTeensy40AS5147P<ENCODER_DIVISIONS, ENCODER_VALUE_COMPRESSION, PWM_WRITE_RESOLUTION>(enc, 3.0, motor_calibration_config, spwm_pin_config, kalman_config); // 3us (micro) sample period

  // Allow skipping ahead a maximum value of 4.0, in terms of the read encoder value measurement, before a skip is detected.
  esc.set_skip_tolerance(4.0);
  // Only allow skipping ahead twice before faulting.
  esc.set_skip_threshold(2);

  // To disable direction enforcement.
  esc.set_direction_enforcement(false);

  // Run setup procedure of the esc. Note this will invoke the encoder's setup method and therefore it is unnecessary to do it explicitly on the encoder instance.
  esc.setup();

  // delay serial read as too early and it gets junk noise data
  while (!Serial.available())
  {
    delay(100);
  }

  // Start sampling.
  esc.start();
}

void loop()
{
  if (esc.started_ok == true)
  {
    esc.loop();
    esc.read_host_control_profile();
  }
}
