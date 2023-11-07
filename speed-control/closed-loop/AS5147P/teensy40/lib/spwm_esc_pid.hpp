#include "spwm_esc.hpp"

namespace kaepek
{
#ifndef KAEPEK_L6234_TEENSY40_AS5147P_PID_ESC
#define KAEPEK_L6234_TEENSY40_AS5147P_PID_ESC

  /**
   * Struct to hold pid config
   */
  struct PIDConfig
  {
    float proportional;
    float differential;
    float integral;
    float power_law_set_point_divisor_cw;
    float power_law_root_cw;
    float power_law_set_point_divisor_ccw;
    float power_law_root_ccw;
    float linear_set_point_coefficient_cw;
    float linear_set_point_coefficient_ccw;
    float linear_bias_cw;
    float linear_bias_ccw;
  };

  /**
   * PidEscL6234Teensy40AS5147P
   *
   * Class to perform SPWM switching via a L6234 motor power supply for a AS5147P rotary encoder on the teensy40 platform with a 4 state physical model.
   * The class implements a simple PID control loop and can use an optional anti cogging map.
   * Based on EscL6234Teensy40AS5147P which uses a sinusoidal voltage (pwm) model.
   */
  template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
  class PidEscL6234Teensy40AS5147P : public EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>
  {
  private:
    // PID coefficients.
    volatile float proportional_coefficient = 0.0;
    volatile float integral_coefficient = 0.0;
    volatile float differential_coefficient = 0.0;

    // PID errors.
    volatile double proportional_error = 0.0;
    volatile double previous_proportional_error = 0.0;
    volatile double differential_error = 0.0;
    volatile double integral_error = 0.0;
    volatile double derivative_error_filtered_1 = 0.0;
    volatile double derivative_error_filtered_2 = 0.0;

    // PID - d term low pass cutoff.
    double desired_derivative_cutoff_frequency = 1500.0;

    // Setpoint.
    volatile float set_point_hz = 0.0;

    // PID bias terms.
    volatile float power_law_set_point_divisor_cw = 0.0;
    volatile float power_law_root_cw = 0.0;
    volatile float power_law_set_point_divisor_ccw = 0.0;
    volatile float power_law_root_ccw = 0.0;
    volatile float linear_set_point_coefficient_cw = 0.0;
    volatile float linear_set_point_coefficient_ccw = 0.0;
    volatile float linear_bias_cw = 0.0;
    volatile float linear_bias_ccw = 0.0;

    // Calculated duty.
    volatile float pid_duty = 0.0;

    using BaseEscClass = EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>;

    /**
     * Method to calculate the temporal numerical derivative.
     * @param value The value to differentiate using the eular method.
     * @param dt The time elapsed.
     * @param previous_value The last value
     */
    double calculate_eular_derivative(double value, double dt, double previous_value);

  public:
    /**
     * PidEscL6234Teensy40AS5147P default constructor.
     */
    PidEscL6234Teensy40AS5147P();

    /**
     * PidEscL6234Teensy40AS5147P constructor with parameters.
     * @param duty_cap The maximum allowable duty. E.g. duty_cap=0.3 represents that the duty cycle can only go to 30% of the largest duty value.
     * @param encoder The digital rotary encoder instance.
     * @param sample_period_microseconds The sample period for the RotaryEncoderSamplerValidator instance to sample the encoder.
     * @param motor_config SPWMMotorConfig for the calibrated bldc motor includes: double cw_zero_displacement_deg, double cw_phase_displacement_deg, double ccw_zero_displacement_deg, ccw_phase_displacement_deg and uint32_t number_of_poles
     * @param spwm_pin_config SPWML6234PinConfig for the LM6234 power circuit includes: uint32_t phase_a, uint32_t phase_b, uint32_t phase_c, uint32_t en, uint32_t frequency
     * @param kalman_config KalmanConfig for the jerk/acceleration/velocity/position model including double alpha, double x_resolution_error, double process_noise
     * @param pid_config PID configuration, including the P, I and D term coefficients. Also including the linear model set_point coefficient and linear bias term (for each direction) OR the power law model set_point divisor and root (for each direction).
     */
    PidEscL6234Teensy40AS5147P(double duty_cap, DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWMMotorConfig motor_config, SPWML6234PinConfig spwm_pin_config, KalmanConfig kalman_config, PIDConfig pid_config);


    /**
     * PidEscL6234Teensy40AS5147P constructor with parameters.
     * @param duty_cap The maximum allowable duty. E.g. duty_cap=0.3 represents that the duty cycle can only go to 30% of the largest duty value.
     * @param encoder The digital rotary encoder instance.
     * @param sample_period_microseconds The sample period for the RotaryEncoderSamplerValidator instance to sample the encoder.
     * @param motor_config SPWMMotorConfig for the calibrated bldc motor includes: double cw_zero_displacement_deg, double cw_phase_displacement_deg, double ccw_zero_displacement_deg, ccw_phase_displacement_deg and uint32_t number_of_poles
     * @param spwm_pin_config SPWML6234PinConfig for the LM6234 power circuit includes: uint32_t phase_a, uint32_t phase_b, uint32_t phase_c, uint32_t en, uint32_t frequency
     * @param kalman_config KalmanConfig for the jerk/acceleration/velocity/position model including double alpha, double x_resolution_error, double process_noise
     * @param pid_config PID configuration, including the P, I and D term coefficients. Also including the linear model set_point coefficient and linear bias term (for each direction) OR the power law model set_point divisor and root (for each direction).
     * @param ac_map_ptr Pointer to an anti-cogging calibration map.
     */
    PidEscL6234Teensy40AS5147P(double duty_cap, DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWMMotorConfig motor_config, SPWML6234PinConfig spwm_pin_config, KalmanConfig kalman_config, PIDConfig pid_config, const int16_t (*ac_map_ptr)[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR]);


    /**
     * Method to read the sampler each microcontroller tick and update the physical jerk model via the Kalman filter with the new measurement sample.
     */
    void loop();

    /**
     * Method to deal with control word input via the serial port.
     */
    void process_host_control_word(uint32_t control_word, uint32_t *data_buffer);

    /**
     * Method to log the values of the communication profile and the physical models to serial out. These include communication profile variables (bool direction and double current_duty_ratio), the physical model components (time, eular_displacement, eular_velocity, eular_acceleration, eular_jerk, kalman_displacement, kalman_velocity, kalman_acceleration and kalman_jerk) and the 3 phase spwm voltages (phase_a, phase_b and phase_c).
     */
    void log();

    /**
     * Method to stop the ESC via stopping the RotaryEncoderSampleValidator, disabling the SPWM pins and stopping the debug logging timer;
     */
    void stop();
  };
#endif
}