#include <Arduino.h>
#include "imxrt.h"
#include "../kalman_four_state/kalman_jerk.hpp"
#include "../encoder/generic/digital_rotary_encoder.hpp"
#include "../encoder/generic/rotary_encoder_sample_validator.hpp"
#include "TeensyTimerTool.h"
#include "../com/comlib.hpp"
/*

Brief:

ESC with SPWM commutations using fitted voltages data directly.

ESC needs to take encoder config and voltage model config.

- It needs to be able to calculate the displacement needed for cw ccw translation of bemf signals
- It needs to be able to start and stop
- It needs a pwm disable feature.
- It needs to take and print kalman vectors
- It needs to read coms from serial port
*/

namespace kaepek
{
#ifndef KAEPEK_L6234_TEENSY40_AS5147P_ESC_DIRECT
#define KAEPEK_L6234_TEENSY40_AS5147P_ESC_DIRECT

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
   * Struct to hold the current duty for each of the 3 phases.
   */
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
   * EscDirectL6234Teensy40AS5147P
   *
   * Class to perform SPWM switching via a L6234 motor power supply for a AS5147P rotary encoder on the teensy40 platform with a 4 state physical model. Class is given
   * a voltage map (pwm duty) pointer and an optional anti-cogging map.
   */
  template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
  class EscDirectL6234Teensy40AS5147P : public RotaryEncoderSampleValidator, public SerialInputControl<4>
  {
  protected:
    // Configurations:

    // Maximum PWM duty cycle.
    static const std::size_t MAX_DUTY = std::pow(2, PWM_WRITE_RESOLUTION) - 1; // take away 1 as starts from 0
    // Maximum value to allow the duty to rise to 0.3 means 30% of the MAX_DUTY
    double duty_cap;
    // L6234 motor driver pin configuration
    SPWML6234PinConfig spwm_pin_config;
    // kalman filter config
    KalmanConfig kalman_config;
    // Const pointer to hold voltage (percentage) for SPWM given direction, channel and compressed encoder steps.
    const int16_t (*voltage_map_ptr)[3][ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR];
    // Const pointer to hold anti cogging duty correction map.
    const int16_t (*ac_map_ptr)[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR];
    // Variable to hold whether anti cogging argument has been given and therefore if it is enables
    bool anti_cogging_enabled = false;
    // Variables to hold SPWM angular resolution (compressed encoder steps) in various datatypes.
    uint32_t spwm_angular_resolution_uint32 = ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR;
    double spwm_angular_resolution_dbl = (double)(ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR);
    double encoder_compression_factor_dbl = ENCODER_COMPRESSION_FACTOR;

    // ESC state variables:

    // Variable to indicate fault status.
    volatile bool fault = false;
    // Variable to indicate  time since last log.
    elapsedMicros micros_since_last_log;
    // Variable to indicate that after a start was attempted did the validator actually start or not.
    bool started = false;
    // Variable to indicate that a start was attempted.
    volatile bool start_attempted = false;
    // Variables to count the number of "loop"'s (aka kalman speed loops) and "samples"'s (encoder samples) within a given time period.
    volatile uint32_t loop_ctr = 0;
    volatile uint32_t sample_ctr = 0;
    // Direction
    RotationDirection direction = RotationDirection::Clockwise;
    volatile bool bl_direction = false;
    volatile byte byte_direction = 0; // UInt8
    // Voltage model current values.
    SPWMVoltageDutyTriplet current_triplet;
    // Current rotor position.
    uint32_t current_encoder_displacement = 0;
    // kalman filter instance
    KalmanJerk1D kalman_filter;
    // Physical model values.
    Dbl4x1 kalman_vec_store = {0};
    Dbl5x1 eular_vec_store = {0};
    // Current duty proportion of the MAX_DUTY, 0.3 would represent 30% of the MAX_DUTY.
    volatile double current_duty_ratio = 0.0;

  public:
    /**
     * EscDirectL6234Teensy40AS5147P default constructor.
     */
    EscDirectL6234Teensy40AS5147P();

    /**
     * EscDirectL6234Teensy40AS5147P constructor with parameters.
     * @param duty_cap The maximum allowable duty. E.g. duty_cap=0.3 represents that the duty cycle can only go to 30% of the largest duty value.
     * @param encoder The digital rotary encoder instance.
     * @param sample_period_microseconds The sample period for the RotaryEncoderSamplerValidator instance to sample the encoder.
     * @param spwm_pin_config SPWML6234PinConfig for the LM6234 power circuit includes: uint32_t phase_a, uint32_t phase_b, uint32_t phase_c, uint32_t en, uint32_t frequency
     * @param kalman_config KalmanConfig for the jerk/acceleration/velocity/position model including double alpha, double x_resolution_error, double process_noise
     * @param voltage_map_ptr Pointer to an array holding for each direction (first index) and for each channel a,b or c (2nd index) and each compressed encoder angle (3rd index) gives a value for the SPWM setting.
     */
    EscDirectL6234Teensy40AS5147P(double duty_cap, DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWML6234PinConfig spwm_pin_config, KalmanConfig kalman_config, const int16_t (*voltage_map_ptr)[3][ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR]);

    /**
     * EscDirectL6234Teensy40AS5147P constructor with parameters.
     * @param duty_cap The maximum allowable duty. E.g. duty_cap=0.3 represents that the duty cycle can only go to 30% of the largest duty value.
     * @param encoder The digital rotary encoder instance.
     * @param sample_period_microseconds The sample period for the RotaryEncoderSamplerValidator instance to sample the encoder.
     * @param spwm_pin_config SPWML6234PinConfig for the LM6234 power circuit includes: uint32_t phase_a, uint32_t phase_b, uint32_t phase_c, uint32_t en, uint32_t frequency
     * @param kalman_config KalmanConfig for the jerk/acceleration/velocity/position model including double alpha, double x_resolution_error, double process_noise
     * @param voltage_map_ptr Pointer to an array holding for each direction (first index) and for each channel a,b or c (2nd index) and each compressed encoder angle (3rd index) gives a value for the SPWM setting.
     * @param ac_map_ptr Pointer to an anti-cogging calibration map.
     */
    EscDirectL6234Teensy40AS5147P(double duty_cap, DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWML6234PinConfig spwm_pin_config, KalmanConfig kalman_config, const int16_t (*voltage_map_ptr)[3][ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR], const int16_t (*ac_map_ptr)[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR]);

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
     * Method to log the values of the communication profile and the physical models to serial out. These include communication profile variables (bool direction and double current_duty_ratio), the physical model components (time, eular_displacement, eular_velocity, eular_acceleration, eular_jerk, kalman_displacement, kalman_velocity, kalman_acceleration and kalman_jerk) and the 3 phase spwm voltages (phase_a, phase_b and phase_c).
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