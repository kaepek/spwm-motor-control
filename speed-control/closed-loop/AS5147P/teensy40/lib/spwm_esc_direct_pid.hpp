#include "spwm_esc_direct.hpp"

namespace kaepek
{
#ifndef KAEPEK_L6234_TEENSY40_AS5147P_PID_ESC_DIRECT
#define KAEPEK_L6234_TEENSY40_AS5147P_PID_ESC_DIRECT

  /**
   * Struct to hold pid config
   */
  struct PIDConfig
  {
    float proportional;
    float differential;
    float integral;
  };

  /**
   * EscDirectL6234Teensy40AS5147P
   *
   * Based on EscDirectL6234Teensy40AS5147P a class to perform SPWM switching via a L6234 motor power supply for a AS5147P rotary encoder on the teensy40 platform with a 4 state physical model,
   * voltages (pwm duties) are fitted directly and given as a pointer. This varient implements a simple PID control loop and can use an optional anti cogging map.
   */
  template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
  class PidEscDirectL6234Teensy40AS5147P : public EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>
  {
  private:
    volatile float proportional_coefficient = 0.0;
    volatile float integral_coefficient = 0.0;
    volatile float differential_coefficient = 0.0;

    // bias terms
    volatile float power_law_set_point_divisor_cw = 0.0;
    volatile float power_law_root_cw = 0.0;
    volatile float power_law_set_point_divisor_ccw = 0.0;
    volatile float power_law_root_ccw = 0.0;

    volatile double proportional_error = 0.0;
    volatile double previous_proportional_error = 0.0;
    volatile double differential_error = 0.0;
    volatile double integral_error = 0.0;

    volatile float set_point = 0.0;

    volatile float pid_duty = 0.0;

    volatile float cache_set_point = 0.0;

    volatile double derivative_error_filtered_1 = 0.0;
    volatile double derivative_error_filtered_2 = 0.0;

    double desired_derivative_cutoff_frequency = 1500.0;

    using BaseEscClass = EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>;

    /**
     * Method to calculate the temporal numerical derivative.
     * @param value The value to differentiate using the eular method.
     * @param dt The time elapsed.
     * @param previous_value The last value
     */
    double calculate_eular_derivative(double value, double dt, double previous_value);

  public:
    /**
     * PidEscDirectL6234Teensy40AS5147P default constructor.
     */
    PidEscDirectL6234Teensy40AS5147P();

    /**
     * PidEscDirectL6234Teensy40AS5147P constructor with parameters.
     * @param encoder The digital rotary encoder instance.
     * @param sample_period_microseconds The sample period for the RotaryEncoderSamplerValidator instance to sample the encoder.
     * @param motor_config SPWMMotorConfig for the calibrated bldc motor includes: double cw_zero_displacement_deg, double cw_phase_displacement_deg, double ccw_zero_displacement_deg, ccw_phase_displacement_deg and uint32_t number_of_poles
     * @param spwm_pin_config SPWML6234PinConfig for the LM6234 power circuit includes: uint32_t phase_a, uint32_t phase_b, uint32_t phase_c, uint32_t en, uint32_t frequency
     * @param kalman_config KalmanConfig for the jerk/acceleration/velocity/position model including double alpha, double x_resolution_error, double process_noise
     * @param voltage_map_ptr Pointer to an array holding for each direction (first index) and for each channel a,b or c (2nd index) and each compressed encoder angle (3rd index) gives a value for the SPWM setting.
     */
    PidEscDirectL6234Teensy40AS5147P(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWML6234PinConfig spwm_pin_config, KalmanConfig kalman_config, PIDConfig pid_config, const int16_t (*voltage_map_ptr)[3][ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR]);

    /**
     * PidEscDirectL6234Teensy40AS5147P constructor with parameters.
     * @param encoder The digital rotary encoder instance.
     * @param sample_period_microseconds The sample period for the RotaryEncoderSamplerValidator instance to sample the encoder.
     * @param motor_config SPWMMotorConfig for the calibrated bldc motor includes: double cw_zero_displacement_deg, double cw_phase_displacement_deg, double ccw_zero_displacement_deg, ccw_phase_displacement_deg and uint32_t number_of_poles
     * @param spwm_pin_config SPWML6234PinConfig for the LM6234 power circuit includes: uint32_t phase_a, uint32_t phase_b, uint32_t phase_c, uint32_t en, uint32_t frequency
     * @param kalman_config KalmanConfig for the jerk/acceleration/velocity/position model including double alpha, double x_resolution_error, double process_noise
     * @param voltage_map_ptr Pointer to an array holding for each direction (first index) and for each channel a,b or c (2nd index) and each compressed encoder angle (3rd index) gives a value for the SPWM setting.
     * @param ac_map_ptr Pointer to an anti-cogging calibration map.
     */
    PidEscDirectL6234Teensy40AS5147P(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWML6234PinConfig spwm_pin_config, KalmanConfig kalman_config, PIDConfig pid_config, const int16_t (*voltage_map_ptr)[3][ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR], const int16_t (*ac_map_ptr)[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR]);

    /**
     * Method to read the sampler each microcontroller tick and update the physical jerk model via the Kalman filter with the new measurement sample.
     */
    void loop();

    /**
     * Method to deal with control word input via the serial port.
     */
    void process_host_control_word(uint32_t control_word, uint32_t *data_buffer);

    /**
     * Method to log the values of the communication profile and the physical models to serial out. These include communication profile variables (bool direction and double com_torque_percentage), the physical model components (time, eular_displacement, eular_velocity, eular_acceleration, eular_jerk, kalman_displacement, kalman_velocity, kalman_acceleration and kalman_jerk) and the 3 phase spwm voltages (phase_a, phase_b and phase_c).
     */
    void log();

    /**
     * Method to stop the ESC via stopping the RotaryEncoderSampleValidator, disabling the SPWM pins and stopping the debug logging timer;
     */
    void stop();
  };
#endif
}