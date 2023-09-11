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
  };

  /**
   * EscL6234Teensy40AS5147P
   *
   * Class to perform SPWM switching via a L6234 motor power supply for a AS5147P rotary encoder on the teensy40 platform with a 4 state physical model.
   */
  template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
  class PidEscL6234Teensy40AS5147P : public EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>
  {
  private:
    float proportional_coefficient = 0.0;
    float differential_coefficient = 0.0;
    float integral_coefficient = 0.0;
    double proportional_error = 0.0;
    double differential_error = 0.0;
    double integral_error = 0.0;

  public:
    /**
     * PidEscL6234Teensy40AS5147P default constructor.
     */
    PidEscL6234Teensy40AS5147P();

    /**
     * PidEscL6234Teensy40AS5147P constructor with parameters.
     * @param encoder The digital rotary encoder instance.
     * @param sample_period_microseconds The sample period for the RotaryEncoderSamplerValidator instance to sample the encoder.
     * @param motor_config SPWMMotorConfig for the calibrated bldc motor includes: double cw_zero_displacement_deg, double cw_phase_displacement_deg, double ccw_zero_displacement_deg, ccw_phase_displacement_deg and uint32_t number_of_poles
     * @param spwm_pin_config SPWML6234PinConfig for the LM6234 power circuit includes: uint32_t phase_a, uint32_t phase_b, uint32_t phase_c, uint32_t en, uint32_t frequency
     * @param kalman_config KalmanConfig for the jerk/acceleration/velocity/position model including double alpha, double x_resolution_error, double process_noise
     */
    PidEscL6234Teensy40AS5147P(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWMMotorConfig motor_config, SPWML6234PinConfig spwm_pin_config, KalmanConfig kalman_config, PIDConfig pid_config);

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
  };
#endif
}