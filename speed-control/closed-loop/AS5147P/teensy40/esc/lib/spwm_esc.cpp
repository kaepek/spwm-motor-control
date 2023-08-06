#include "spwm_esc.hpp"
#include "../kalman_four_state/kalman_jerk.cpp"
#include "../encoder/digital_rotary_encoder.cpp"
#include "../encoder/generic/rotary_encoder_sample_validator.cpp"
#include "spwm_voltage_model_discretiser.cpp"
using namespace TeensyTimerTool;

PeriodicTimer logging_timer(GPT2);

namespace kaepek
{
    /*

       template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t MAX_DUTY>
    uint32_t SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>::raw_encoder_value_to_compressed_encoder_value(double raw_encoder_value)

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t MAX_DUTY>
    SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>::SPWMVoltageModelDiscretiser(){

    };

   EscTeensy40AS5147P<std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>::
    */

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    EscTeensy40AS5147P<std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>::EscTeensy40AS5147P() : RotaryEncoderSampleValidator()
    {
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    EscTeensy40AS5147P<std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>::EscTeensy40AS5147P(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWMMotorConfig motor_config, SPWMPinConfig spwm_ping_config, KalmanConfig kalman_config) : RotaryEncoderSampleValidator(encoder, sample_period_microseconds)
    {
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    uint32_t EscTeensy40AS5147P<std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>::apply_phase_displacement(double encoder_value)
    {
        // cw_displacement_deg
        // ccw_displacement_deg
        /*
          Translating between electrical and mechanical degrees
          Electrical angle = P\2 (mechanical angle)
          So (2 * Electrical angle) / P = mechanical angle
          P is poles (NOT POLE PAIRS)
        */
        // dont forget you can transition over 0 or 16384 so need to fnmod this value before returning.
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscTeensy40AS5147P<std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>::read_host_control_profile()
    {
        // read profile
        // set direction / thrust
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscTeensy40AS5147P<std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>::post_sample_logic(uint32_t encoder_value)
    {
        // take encoder value
        // apply displacement
        // convert to compressed
        // get triplet
        // apply triplet
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscTeensy40AS5147P<std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>::post_fault_logic(RotaryEncoderSampleValidator::Fault fault_code)
    {
        // force stop
        // shut off motor pins
        // disable logging
        // print debug message
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscTeensy40AS5147P<std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>::setup()
    {
        // set pwm pins for output
        // set pwm write resolution
        // invoke base method setup
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscTeensy40AS5147P<std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>::loop()
    {
        /*
          if (started_ok == true)
  {
    //readHostControlProfile


    // Check the encoder has a new sample.
    if (esc.has_new_sample() == true)
    {
      // Define variables to store the sampled encoder value and the number of elapsed microseconds since the last samples retrieval.
      uint32_t encoder_value;
      uint32_t elapsed_micros_since_last_sample;
      // Fetch the stored values from the buffer.
      esc.get_sample_and_elapsed_time(encoder_value, elapsed_micros_since_last_sample);
      // Convert microseconds to seconds.
      double seconds_since_last = (double) elapsed_micros_since_last_sample * (double) 1e-6;
      // Perform one kalman step with the data.
      filter.step(seconds_since_last, encoder_value);
      // Extract state values.
      double *kalman_vec = filter.get_kalman_vector();
      double *eular_vec = filter.get_eular_vector();
      // Store state in cache ready for printing.
      cli();
      kalman_vec_store[0] = kalman_vec[0];
      kalman_vec_store[1] = kalman_vec[1];
      kalman_vec_store[2] = kalman_vec[2];
      kalman_vec_store[3] = kalman_vec[3];
      eular_vec_store[0] = eular_vec[0];
      eular_vec_store[1] = eular_vec[1];
      eular_vec_store[2] = eular_vec[2];
      eular_vec_store[3] = eular_vec[3];
      eular_vec_store[4] = eular_vec[4];
      sei();
    }
  }
  else
  {
    // If the esc did not start in a good state, then print the configuration issues out via the serial port, by invoking the print_configuration_issues method of the esc.
    esc.print_configuration_issues();
    delayMicroseconds(10'000'000);
  }
        */
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscTeensy40AS5147P<std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>::start()
    {
        // start esc
        // start logging timer
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscTeensy40AS5147P<std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>::stop()
    {
        // stop esc
        // stop logging timer
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscTeensy40AS5147P<std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>::log()
    {
        Serial.print(eular_vec[0]);
        Serial.print(",");
        Serial.print(eular_vec[1]);
        Serial.print(",");
        Serial.print(eular_vec[2]);
        Serial.print(",");
        Serial.print(eular_vec[3]);
        Serial.print(",");
        Serial.print(eular_vec[4]);
        Serial.print(",");
        Serial.print(kalman_vec[0]);
        Serial.print(",");
        Serial.print(kalman_vec[1]);
        Serial.print(",");
        Serial.print(kalman_vec[2]);
        Serial.print(",");
        Serial.print(kalman_vec[3]);
        Serial.print(",");
        Serial.print(current_triplet.a);
        Serial.print(",");
        Serial.print(current_triplet.b);
        Serial.print(",");
        Serial.print(current_triplet.c);
    }

}