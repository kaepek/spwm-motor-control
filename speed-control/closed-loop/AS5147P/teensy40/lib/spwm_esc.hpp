#include <Arduino.h>
#include "imxrt.h"
#include "../kalman_four_state/kalman_jerk.hpp"
#include "../encoder/generic/digital_rotary_encoder.hpp"
#include "../encoder/generic/rotary_encoder_sample_validator.hpp"
#include "TeensyTimerTool.h"
// #include "spwm_voltage_model_discretiser.hpp"
#include "../com/comlib.hpp"
/*

Brief:

ESC with SPWM commutations.

ESC needs to take encoder config and PWM config.

- It needs to be able to calculate the displacement needed for cw ccw translation of bemf signals
- It needs to be able to start and stop
- It needs a pwm disable feature.
- It needs to take and print kalman vectors
- It needs to read coms from serial port
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

  struct SPWMVoltageDutyTriplet
  {
    uint32_t phase_a;
    uint32_t phase_b;
    uint32_t phase_c;
  };

  /**
   * RotationDirection enum class type:
   * - CounterClockwise
   * - Clockwise
   */
  enum class RotationDirection
  {
    CounterClockwise = 0,
    Clockwise = 1
  };

  /**
   * EscL6234Teensy40AS5147P
   *
   * Class to perform SPWM switching via a L6234 motor power supply for a AS5147P rotary encoder on the teensy40 platform with a 4 state physical model.
   */
  template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
  class EscL6234Teensy40AS5147P : public RotaryEncoderSampleValidator, public SerialInputControl<4>
  {
  protected:
    // constants
    // static constexpr double cw_displacement_deg = 0.0;
    // static constexpr double ccw_displacement_deg = 0.0;
    // static constexpr float log_frequency_micros = 5000;
    static const std::size_t MAX_DUTY = std::pow(2, PWM_WRITE_RESOLUTION) - 1; // take away 1 as starts from 0
    // static const int size_of_host_profile = 3;
    // discretiser
    RotationDirection direction = RotationDirection::Clockwise;
    // typename SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>::Direction discretiser_direction;
    // SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY> discretiser;
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
    // Host communication buffer to store the thrust/direction profile (3 bytes in total) from the serial stream
    // char host_profile_buffer[size_of_host_profile] = {0, 0, 0};
    // int host_profile_buffer_ctr = 0;
    // Host communication variables
    volatile double com_torque_percentage = 0.0;
    volatile byte com_direction_value = 0; // UInt8
    // ESC state variables
    // Variable to indicate fault status
    volatile bool fault = false;
    // Variable to indicate  time since last log
    elapsedMicros micros_since_last_log;
    // Variable to indicate that after a start was attempted did the validator actually start or not
    bool started = false;
    // Variable to indicate that a start was attempted.
    volatile bool start_attempted = false;
    // Variables to count the number of "loop"'s (aka kalman speed loops) and "samples"'s (encoder samples) within a given time period.
    volatile uint32_t loop_ctr = 0;
    volatile uint32_t sample_ctr = 0;

    float cw_phase_a_lookup[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0};
    float cw_phase_b_lookup[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0};
    float cw_phase_c_lookup[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0};
    float ccw_phase_a_lookup[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0};
    float ccw_phase_b_lookup[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0};
    float ccw_phase_c_lookup[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0};

    // float cw_corrections[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0}; // wanted float but wont compile
    // float ccw_corrections[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0};

    uint32_t spwm_angular_resolution_uint32 = ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR;
    double spwm_angular_resolution_dbl = (double)(ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR);
    double encoder_compression_factor_dbl = ENCODER_COMPRESSION_FACTOR;
    uint32_t number_of_poles;
    double cw_zero_displacement_deg;
    double cw_phase_displacement_deg;
    double ccw_zero_displacement_deg;
    double ccw_phase_displacement_deg;

    // float cw_corrections[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0}; // this works!
    // float ccw_corrections[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0};

  public:
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
    EscL6234Teensy40AS5147P(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWMMotorConfig motor_config, SPWML6234PinConfig spwm_pin_config, KalmanConfig kalman_config);

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
    virtual void loop();

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
    virtual void log();

    /**
     * Method to deal with control word input via the serial port.
     */
    virtual void process_host_control_word(uint32_t control_word, uint32_t *data_buffer);

    /**
     * Method to get the fault status
     */
    bool get_fault_status();

    /**
     * Method to get started status
     */
    bool get_started_status();

  protected:
    /**
     * Method to update cw_zero_displacement_deg.
     * @param value The value to update.
     */
    void set_cw_zero_displacement_deg(float value);

    /**
     * Method to update ccw_zero_displacement_deg.
     * @param value The value to update.
     */
    void set_ccw_zero_displacement_deg(float value);

    /**
     * Method to update cw_phase_displacement_deg.
     * @param value The value to update.
     */
    void set_cw_phase_displacement_deg(float value);

    /**
     * Method to update ccw_phase_displacement_deg.
     * @param value The value to update.
     */
    void set_ccw_phase_displacement_deg(float value);

    /**
     * Method to update the trig tables when a phase or zero displacement has been changed.
     */
    void update_lookup_tables();

    /**
     * raw_encoder_value_to_compressed_encoder_value default constructor.
     * @param raw_encoder_value The encoder value as read by the sensor
     * @return Returns the encoder value compressed by the ENCODER_COMPRESSION_FACTOR
     */
    uint32_t raw_encoder_value_to_compressed_encoder_value(double raw_encoder_value);

    /**
     * Method to get the current phase a,b and c duty triplet.
     * @param current_duty The current duty magnitude.
     * @param encoder_current_compressed_displacement The current encoder displacement measurement value compressed by the compression factor.
     * @return a SPWMVoltageDutyTriplet struct with the 3 phase duties.
     */
    SPWMVoltageDutyTriplet get_pwm_triplet(double current_duty, uint32_t encoder_current_compressed_displacement, RotationDirection direction);

    /**
     * Method to get radians from degrees.
     * @param deg The number of degrees.
     * @return The number of radians.
     */
    static double deg_to_rad(double deg);

    /**
     * Method to get degrees from radians.
     * @param rad The number of radians.
     * @return The number of degrees.
     */
    static double rad_to_deg(double rad);

    /**
     * Method to apply modulus in a way which obeys negative values
     * @param value The number of to perform the modulo on.
     * @param mod The base for the modulo.
     * @return The number modulo the base
     */
    static double fnmod(double value, double mod);
  };
#endif
}