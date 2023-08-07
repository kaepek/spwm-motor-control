#include <Arduino.h>
#include "imxrt.h"
#include "../kalman_four_state/kalman_jerk.hpp"
#include "../encoder/digital_rotary_encoder.hpp"
#include "../encoder/generic/rotary_encoder_sample_validator.hpp"
#include "TeensyTimerTool.h"
#include "spwm_voltage_model_discretiser.hpp"

/*

Brief:

ESC with SPWM commutations

esc needs to take encoder config and PWM config
it needs to be able to calculate the displacement needed for cw ccw translation of bemf signals

it needs to be able to start and stop
needs a pwm disable feature.

take and print kalman vectors

read coms from serial port

*/

/**
 * EscTeensy40AS5147P
 *
 * Class to perform SPWM switching and speed calculations for a AS5147P rotary encoder on the teensy40 platform.
 */

/**
 * double cw_zero_displacement_deg = -1.86;
double cw_phase_displacement_deg = 240.01;
double ccw_zero_displacement_deg = 15.31;
double ccw_phase_displacement_deg = 119.99;
uint32_t number_of_poles = 14;

*/
namespace kaepek
{
  struct SPWMMotorConfig
  {
    double cw_zero_displacement_deg;
    double cw_phase_displacement_deg;
    double ccw_zero_displacement_deg;
    double ccw_phase_displacement_deg;
    uint32_t number_of_poles;
  };

  struct KalmanConfig
  {
    double alpha;
    double angular_resolution_error;
    double process_noise;
  };

  struct SPWMPinConfig
  {
    uint32_t phase_a;
    uint32_t phase_b;
    uint32_t phase_c;
    uint32_t en;
    uint32_t frequency;
  };

  template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
  class EscTeensy40AS5147P : public RotaryEncoderSampleValidator
  {
  private:
    const std::size_t MAX_DUTY = std::pow(2, PWM_WRITE_RESOLUTION) - 1; // take away 1 as starts from 0
    SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY> discretiser;
    Dbl4x1 kalman_vec_store = { 0 };
    Dbl5x1 eular_vec_store = { 0 };
    SPWMVoltageDutyTriplet current_triplet;
    const double cw_displacement_deg = 60.0;
    const double ccw_displacement_deg = -60.0;

    uint32_t current_encoder_displacement = 0;

    SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>::Direction discretiser_direction;

    volatile uint16_t com_torque_value = 0;        // UInt16LE
    volatile unsigned int com_direction_value = 0; // UInt8

    uint32_t apply_phase_displacement(double encoder_value);
    void read_host_control_profile();

    bool started_ok = false;

    SPWMMotorConfig motor_config;
    SPWMPinConfig spwm_pin_config;
    KalmanConfig kalman_config;

    KalmanJerk1D kalman_filter;

    const float log_frequency_micros = 100;

    // we read 3 bytes in total
    const int size_of_host_profile = 3;
    // buffer to store the thrust/direction profile from the serial stream
    char host_profile_buffer[size_of_host_profile] = {0, 0, 0};
    int host_profile_buffer_ctr = 0;

  public:
    // Default constuctor.
    EscTeensy40AS5147P(); // : RotaryEncoderSampleValidator();

    // Constructor with parameters.
    EscTeensy40AS5147P(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWMMotorConfig motor_config, SPWMPinConfig spwm_ping_config, KalmanConfig kalman_config); // : RotaryEncoderSampleValidator(encoder, sample_period_microseconds);

    void post_sample_logic(uint32_t encoder_value);

    void post_fault_logic(RotaryEncoderSampleValidator::Fault fault_code);

    void setup();

    void loop();

    bool start();

    void stop();

    void log();
  };
}