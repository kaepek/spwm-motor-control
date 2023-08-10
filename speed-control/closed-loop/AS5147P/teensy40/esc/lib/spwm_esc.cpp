#include "spwm_esc.hpp"
#include "../kalman_four_state/kalman_jerk.cpp"
#include "../encoder/digital_rotary_encoder.cpp"
#include "../encoder/generic/rotary_encoder_sample_validator.cpp"
#include "spwm_voltage_model_discretiser.cpp"
using namespace TeensyTimerTool;

#ifndef DISABLE_SPWM_PIN_MODIFICATION
#define DISABLE_SPWM_PIN_MODIFICATION true
#endif

PeriodicTimer logging_timer(GPT2);

namespace kaepek
{

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::EscL6234Teensy40AS5147P() : RotaryEncoderSampleValidator()
    {
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::EscL6234Teensy40AS5147P(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWMMotorConfig motor_config, SPWMPinConfig spwm_pin_config, KalmanConfig kalman_config) : RotaryEncoderSampleValidator(encoder, sample_period_microseconds)
    {
        this->motor_config = motor_config;
        this->spwm_pin_config = spwm_pin_config;
        this->kalman_config = kalman_config;
        this->discretiser = SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>(motor_config.cw_zero_displacement_deg, motor_config.cw_phase_displacement_deg, motor_config.ccw_zero_displacement_deg, motor_config.ccw_phase_displacement_deg, motor_config.number_of_poles);
        this->kalman_filter = KalmanJerk1D(kalman_config.alpha, kalman_config.angular_resolution_error, kalman_config.process_noise, true, (double)ENCODER_DIVISIONS);
        logging_timer.begin([this]
                            { this->log(); },
                            this->log_frequency_micros, false);
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    uint32_t EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::apply_phase_displacement(double encoder_value)
    {
        double electrical_displacement_deg = 0.0;
        if (this->discretiser_direction == SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>::Direction::Clockwise)
        {
            // clockwise
            electrical_displacement_deg = cw_displacement_deg;
        }
        else
        {
            // counter clockwise
            electrical_displacement_deg = ccw_displacement_deg;
        }

        // Translating between electrical and mechanical degrees
        // Electrical angle = P\2 (mechanical angle)
        // So (2 * Electrical angle) / P = mechanical angle
        // P is poles (NOT POLE PAIRS)

        double mechanical_displacement_deg = (2.0 * electrical_displacement_deg) / (double)motor_config.number_of_poles;
        double mechanical_displacement_steps = (mechanical_displacement_deg / 360.0) * (double)ENCODER_DIVISIONS;
        double translated_mechanical_steps = encoder_value + mechanical_displacement_steps;

        // dont forget you can transition over 0 or 16384 so need to fnmod this value before returning.

        uint32_t final_encoder_value = SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>::fnmod(translated_mechanical_steps, ENCODER_DIVISIONS);

        return final_encoder_value;
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    bool EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::read_host_control_profile()
    {
        // read profile
        bool processed_a_full_profile = false;
        cli(); // no interrupt
        while (Serial.available())
        {
            host_profile_buffer[host_profile_buffer_ctr] = Serial.read(); // read byte from usb
            host_profile_buffer_ctr++;                                    // in buffer
            if (host_profile_buffer_ctr % size_of_host_profile == 0)
            { // when we have the right number of bytes for the whole input profile
                com_torque_value = (host_profile_buffer[1] << 8) | host_profile_buffer[0];
                // clamp value to max
                com_torque_value = com_torque_value > MAX_DUTY ? MAX_DUTY : com_torque_value;
                // extract direction from buffer (0 is cw 1 is ccw)
                com_direction_value = host_profile_buffer[2];
                // set direction / thrust
                if (com_direction_value == 0)
                {
                    discretiser_direction = SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>::Direction::Clockwise;
                    set_direction(RotaryEncoderSampleValidator::Direction::Clockwise); // update validated direction ignored if set_direction_enforcement(false)
                }
                else
                {
                    discretiser_direction = SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>::Direction::CounterClockwise;
                    set_direction(RotaryEncoderSampleValidator::Direction::CounterClockwise); // update validated direction ignored if set_direction_enforcement(false)
                }
                // indicate we have processed a full profile
                processed_a_full_profile = true;
            }
            host_profile_buffer_ctr %= size_of_host_profile; // reset buffer ctr for a new profile
        }
        sei(); // interrupt
        return processed_a_full_profile;
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::post_sample_logic(uint32_t encoder_value)
    {
        // take encoder value
        // apply displacement
        uint32_t displaced_encoder_value = apply_phase_displacement(encoder_value);
        // convert to compressed
        uint32_t compressed_encoder_value = discretiser.raw_encoder_value_to_compressed_encoder_value(displaced_encoder_value);
        // get triplet
        // apply triplet
        current_triplet = discretiser.get_pwm_triplet(com_torque_value, compressed_encoder_value, discretiser_direction);
        // set pin values
#if !DISABLE_SPWM_PIN_MODIFICATION
        // This section of code will be disabled when DISABLE_SPWM_PIN_MODIFICATION is true.
        analogWrite(spwm_pin_config.phase_a, current_triplet.phase_a);
        analogWrite(spwm_pin_config.phase_b, current_triplet.phase_b);
        analogWrite(spwm_pin_config.phase_c, current_triplet.phase_c);
#endif
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::post_fault_logic(RotaryEncoderSampleValidator::Fault fault_code)
    {
        // force stop
        // shut off motor pins
        // disable logging
        // print debug message
        stop();

        Serial.println("Experienced a fault shutting down");
    }

    /*
    #if !DISABLE_SPWM_PIN_MODIFICATION
        // This section of code will be disabled when DISABLE_SPWM_PIN_MODIFICATION is true.
        #endif
    */

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::setup()
    {
        // set pwm pins for output
        pinMode(spwm_pin_config.phase_a, OUTPUT);
        pinMode(spwm_pin_config.phase_b, OUTPUT);
        pinMode(spwm_pin_config.phase_c, OUTPUT);
        pinMode(spwm_pin_config.en, OUTPUT);

        // set pwm write resolution
        analogWriteRes(PWM_WRITE_RESOLUTION);

        // set pwm frequency
        analogWriteFrequency(spwm_pin_config.phase_a, spwm_pin_config.frequency);
        analogWriteFrequency(spwm_pin_config.phase_b, spwm_pin_config.frequency);
        analogWriteFrequency(spwm_pin_config.phase_c, spwm_pin_config.frequency);

        // turn off all spwm pins
        digitalWrite(spwm_pin_config.en, LOW);
        digitalWrite(spwm_pin_config.phase_a, LOW);
        digitalWrite(spwm_pin_config.phase_b, LOW);
        digitalWrite(spwm_pin_config.phase_c, LOW);

        // init the current phase triplet as 0
        current_triplet.phase_a = 0;
        current_triplet.phase_b = 0;
        current_triplet.phase_c = 0;

        // setip the rotary encoder sample (will init the encoder).
        RotaryEncoderSampleValidator::setup();
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::loop()
    {

        if (started_ok == true)
        {
            // Check the encoder has a new sample.
            if (has_new_sample() == true)
            {
                // Define variables to store the sampled encoder value and the number of elapsed microseconds since the last samples retrieval.
                uint32_t encoder_value;
                uint32_t elapsed_micros_since_last_sample;
                // Fetch the stored values from the buffer.
                get_sample_and_elapsed_time(encoder_value, elapsed_micros_since_last_sample);
                // Convert microseconds to seconds.
                double seconds_since_last = (double)elapsed_micros_since_last_sample * (double)1e-6;
                // Perform one kalman step with the data.
                kalman_filter.step(seconds_since_last, encoder_value);
                // Extract state values.
                double *kalman_vec = kalman_filter.get_kalman_vector();
                double *eular_vec = kalman_filter.get_eular_vector();
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
            print_configuration_issues();
            delayMicroseconds(10'000'000);
        }
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    bool EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::start()
    {
#if !DISABLE_SPWM_PIN_MODIFICATION
        // This section of code will be disabled when DISABLE_SPWM_PIN_MODIFICATION is true.
        digitalWrite(spwm_pin_config.en, HIGH);
#endif

        started_ok = RotaryEncoderSampleValidator::start();

        logging_timer.start();

        return started_ok;
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::stop()
    {
        RotaryEncoderSampleValidator::stop();

        digitalWrite(spwm_pin_config.en, LOW);
        digitalWrite(spwm_pin_config.phase_a, LOW);
        digitalWrite(spwm_pin_config.phase_b, LOW);
        digitalWrite(spwm_pin_config.phase_c, LOW);

        logging_timer.stop();
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::log()
    {
        Serial.print(eular_vec_store[0]);
        Serial.print(",");
        Serial.print(eular_vec_store[1]);
        Serial.print(",");
        Serial.print(eular_vec_store[2]);
        Serial.print(",");
        Serial.print(eular_vec_store[3]);
        Serial.print(",");
        Serial.print(eular_vec_store[4]);
        Serial.print(",");
        Serial.print(kalman_vec_store[0]);
        Serial.print(",");
        Serial.print(kalman_vec_store[1]);
        Serial.print(",");
        Serial.print(kalman_vec_store[2]);
        Serial.print(",");
        Serial.print(kalman_vec_store[3]);
        Serial.print(",");
        Serial.print(current_triplet.phase_a);
        Serial.print(",");
        Serial.print(current_triplet.phase_b);
        Serial.print(",");
        Serial.print(current_triplet.phase_c);
    }

}