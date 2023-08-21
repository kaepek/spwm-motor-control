#include <Arduino.h>
#include "imxrt.h"
#include "../kalman_four_state/kalman_jerk.hpp"
#include "../encoder/generic/digital_rotary_encoder.hpp"
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
 * double cw_zero_displacement_deg = -1.86;
double cw_phase_displacement_deg = 240.01;
double ccw_zero_displacement_deg = 15.31;
double ccw_phase_displacement_deg = 119.99;
uint32_t number_of_poles = 14;

*/
namespace kaepek
{
#ifndef KAEPEK_L6234_TEENSY40_AS5147P_ESC
#define KAEPEK_L6234_TEENSY40_AS5147P_ESC

  /**
   * Struct to hold a bldc bemf voltage model.
   */
  struct SPWMMotorConfig
  {
    double cw_zero_displacement_deg;
    double cw_phase_displacement_deg;
    double ccw_zero_displacement_deg;
    double ccw_phase_displacement_deg;
    uint32_t number_of_poles;
  };

  /**
   * Struct to hold the config for the 4 state Kalman filter. Allows for measuring velocity / acceleration / jerk of the motor.
   */
  struct KalmanConfig
  {
    double alpha;
    double x_resolution_error;
    double process_noise;
  };

  /**
   * Stuct to hold the pin outs for the L6234 SPWM motor driver.
   */
  struct SPWML6234PinConfig
  {
    uint32_t phase_a;
    uint32_t phase_b;
    uint32_t phase_c;
    uint32_t en;
    uint32_t frequency;
  };

  /**
   * EscL6234Teensy40AS5147P
   *
   * Class to perform SPWM switching via a L6234 motor power supply for a AS5147P rotary encoder on the teensy40 platform with a 4 state physical model.
   */
  template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
  class EscL6234Teensy40AS5147P : public RotaryEncoderSampleValidator
  {
  private:
    // constants
    static constexpr double cw_displacement_deg = 0.0;
    static constexpr double ccw_displacement_deg = 0.0;
    static constexpr float log_frequency_micros = 5000;
    static const std::size_t MAX_DUTY = std::pow(2, PWM_WRITE_RESOLUTION) - 1; // take away 1 as starts from 0
    static const int size_of_host_profile = 3;
    static const int led_pin = 13;
    // discretiser
    typename SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>::Direction discretiser_direction;
    SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY> discretiser;
    // physical model values
    Dbl4x1 kalman_vec_store = {0};
    Dbl5x1 eular_vec_store = {0};
    // voltage model values
    SPWMVoltageDutyTriplet current_triplet;
    uint32_t current_encoder_displacement = 0;
    // motor calibration config
    SPWMMotorConfig motor_config;
    // L6234 motor driver pin configuration
    SPWML6234PinConfig spwm_pin_config;
    // kalman filter config
    KalmanConfig kalman_config;
    // kalman filter instance
    KalmanJerk1D kalman_filter;
    // host communication buffer to store the thrust/direction profile (3 bytes in total) from the serial stream
    char host_profile_buffer[size_of_host_profile] = {0, 0, 0};
    int host_profile_buffer_ctr = 0;
    // host communication variables
    // volatile uint16_t com_torque_value = 0; // UInt16LE
    volatile double com_torque_percentage = 0.0;
    volatile byte com_direction_value = 0; // UInt8
    // esc state variable
    volatile bool fault = false;

    elapsedMicros micros_since_last_log;

    /**
     * Method to calculate the required displacement of the encoder value such that the encoder value is displaced from the calibration models bemf recording such that
     * the motor and rotor magnetic fields ~90 degrees seperated, thus allowing for peak efficiency.
     */
    uint32_t apply_phase_displacement(double encoder_value);

  public:
    bool started_ok = false;
    volatile uint32_t loop_ctr = 0;
    volatile uint32_t sample_ctr = 0;

    /**
     * EscL6234Teensy40AS5147P default constructor.
     */
    EscL6234Teensy40AS5147P();

    /**
     * EscL6234Teensy40AS5147P constructor with parameters.
     * @param encoder The digital rotary encoder instance.
     * @param sample_period_microseconds The sample period for the RotaryEncoderSamplerValidator instance to sample the encoder.
     * @param motor_config SPWMMotorConfig for the calibrated bldc motor includes: double cw_zero_displacement_deg, double cw_phase_displacement_deg, double ccw_zero_displacement_deg, ccw_phase_displacement_deg and uint32_t number_of_poles
     * @param spwm_pin_config SPWML6234PinConfig for the LM6234 power circuit includes: uint32_t phase_a, uint32_t phase_b, uint32_t phase_c, uint32_t en, uint32_t frequency
     * @param kalman_config KalmanConfig for the jerk/acceleration/velocity/position model including double alpha, double x_resolution_error, double process_noise
     */
    EscL6234Teensy40AS5147P(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWMMotorConfig motor_config, SPWML6234PinConfig spwm_pin_config, KalmanConfig kalman_config); // : RotaryEncoderSampleValidator(encoder, sample_period_microseconds);

    /**
     * Method set the SPWM pin voltages from the SPWMVoltageModelDiscretiser when a successful encoder sample is taken and is determined to be a plausible value and the value indicates motion which obeys the direction constraint (if direction enforcement has been enabled).
     *
     * @param encoder_value The latest validated encoder measurement sample value.
     */
    void post_sample_logic(uint32_t encoder_value);

    /**
     * Method to shut down the SPWM pins and ESc when the EncoderTracker detects an inplausabile measurement sample value or the motion of the encoder is determined to violate the motion direction contraint (if direction enforcement has been enabled).
     * @param fault_code RotaryEncoderSampleValidator::Fault code either WrongDirection or SkippedSteps
     */
    void post_fault_logic(RotaryEncoderSampleValidator::Fault fault_code);

    /**
     * Method to setup the ESC SPWM motor pins and digital encoder such that it is ready to read encoder measurement values and apply the correct voltages.
     */
    void setup();

    /**
     * Method to read the sampler each microcontroller tick and update the physical jerk model via the Kalman filter with the new measurement sample.
     */
    void loop();

    /**
     * Method to start the ESC via starting the RotaryEncoderSampleValidator via its start method (therefore encoder readings will start being read and internal voltage and physical models will start updating) and starting the logging timer to print diagnostics of the voltage and physical models.
     */
    bool start();

    /**
     * Method to stop the ESC via stopping the RotaryEncoderSampleValidator, disabling the SPWM pins and stopping the debug logging timer;
     */
    void stop();

    /**
     * Method to log the values of the communication profile and the physical models to serial out. These include communication profile variables (bool direction and double com_torque_percentage), the physical model components (time, eular_displacement, eular_velocity, eular_acceleration, eular_jerk, kalman_displacement, kalman_velocity, kalman_acceleration and kalman_jerk) and the 3 phase spwm voltages (phase_a, phase_b and phase_c).
     */
    void log();

    /**
     * Method to read the serial port for changes in the host control profile. Profile encodes the direction of the motor and the torque value.
     */
    bool read_host_control_profile();

    /**
     * Method to get the fault status
     */
    bool get_fault_status();
  };
#endif
}