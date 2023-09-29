#include "../com/comlib.cpp"
#include "spwm_esc_direct.hpp"
#include "../kalman_four_state/kalman_jerk.cpp"
#include "../encoder/digital_rotary_encoder.cpp"
#include "../encoder/generic/rotary_encoder_sample_validator.cpp"

using namespace TeensyTimerTool;

#ifndef DISABLE_SPWM_PIN_MODIFICATION
#define DISABLE_SPWM_PIN_MODIFICATION false
#endif

#ifndef ENABLE_VERBOSE_LOGGING
#define ENABLE_VERBOSE_LOGGING true
#endif

// PeriodicTimer logging_timer(GPT2);

namespace kaepek
{

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::EscDirectL6234Teensy40AS5147P() : RotaryEncoderSampleValidator(), SerialInputControl<4>()
    {
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::EscDirectL6234Teensy40AS5147P(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWML6234PinConfig spwm_pin_config, KalmanConfig kalman_config, const int16_t (*voltage_map_ptr)[3][ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR]) : RotaryEncoderSampleValidator(encoder, sample_period_microseconds), SerialInputControl<4>()
    {
        this->spwm_pin_config = spwm_pin_config;
        this->kalman_config = kalman_config;
        this->voltage_map_ptr = voltage_map_ptr;
        this->kalman_filter = KalmanJerk1D(kalman_config.alpha, kalman_config.x_resolution_error, kalman_config.process_noise, true, (double)ENCODER_DIVISIONS);
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::EscDirectL6234Teensy40AS5147P(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWML6234PinConfig spwm_pin_config, KalmanConfig kalman_config, const int16_t (*voltage_map_ptr)[3][ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR], const int16_t (*ac_map_ptr)[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR]) : RotaryEncoderSampleValidator(encoder, sample_period_microseconds), SerialInputControl<4>()
    {
        this->spwm_pin_config = spwm_pin_config;
        this->kalman_config = kalman_config;
        this->kalman_filter = KalmanJerk1D(kalman_config.alpha, kalman_config.x_resolution_error, kalman_config.process_noise, true, (double)ENCODER_DIVISIONS);
        this->ac_map_ptr = ac_map_ptr;
        this->voltage_map_ptr = voltage_map_ptr;
        this->anti_cogging_enabled = true;
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::post_sample_logic(uint32_t encoder_value)
    {
        sample_ctr++;

        // Take encoder value.
        this->current_encoder_displacement = encoder_value;
        // Convert to compressed.
        uint32_t compressed_encoder_value = raw_encoder_value_to_compressed_encoder_value(encoder_value);
        // Get and apply triplet.
        current_triplet = get_pwm_triplet(com_torque_percentage, compressed_encoder_value, direction); // * (double)MAX_DUTY
                                                                                                       // Set pin values.
#if !DISABLE_SPWM_PIN_MODIFICATION
        // This section of code will be disabled when DISABLE_SPWM_PIN_MODIFICATION is true.
        analogWrite(spwm_pin_config.phase_a, current_triplet.phase_a);
        analogWrite(spwm_pin_config.phase_b, current_triplet.phase_b);
        analogWrite(spwm_pin_config.phase_c, current_triplet.phase_c);
#endif
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::post_fault_logic(RotaryEncoderSampleValidator::Fault fault_code)
    {
        // Force stop.
        // Set fault flag.
        this->fault = true;
        // Shut off motor pins.
        stop();
        // Print fault message.
        Serial.print("Error SkippedSteps: ");
        Serial.println(fault_code == RotaryEncoderSampleValidator::Fault::SkippedSteps);
        Serial.print("Error WrongDirection: ");
        Serial.println(fault_code == RotaryEncoderSampleValidator::Fault::WrongDirection);
        Serial.println("Experienced a fault shutting down");
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::setup()
    {
#if !DISABLE_SPWM_PIN_MODIFICATION
        // Set pwm pins for output.
        pinMode(spwm_pin_config.phase_a, OUTPUT);
        pinMode(spwm_pin_config.phase_b, OUTPUT);
        pinMode(spwm_pin_config.phase_c, OUTPUT);
        pinMode(spwm_pin_config.en, OUTPUT);

        // Set pwm write resolution.
        analogWriteRes(PWM_WRITE_RESOLUTION);

        // Set pwm frequency.
        analogWriteFrequency(spwm_pin_config.phase_a, spwm_pin_config.frequency);
        analogWriteFrequency(spwm_pin_config.phase_b, spwm_pin_config.frequency);
        analogWriteFrequency(spwm_pin_config.phase_c, spwm_pin_config.frequency);

        // Turn off all spwm pins.
        digitalWrite(spwm_pin_config.en, LOW);
        digitalWrite(spwm_pin_config.phase_a, LOW);
        digitalWrite(spwm_pin_config.phase_b, LOW);
        digitalWrite(spwm_pin_config.phase_c, LOW);
#endif

        // Init the current phase triplet as 0.
        current_triplet.phase_a = 0;
        current_triplet.phase_b = 0;
        current_triplet.phase_c = 0;

        // Setup the rotary encoder sample (will init the encoder).
        RotaryEncoderSampleValidator::setup();
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::loop()
    {

        if (started == true)
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
                loop_ctr++;
                sei();
            }
        }
        else if (start_attempted == true)
        {
            // If the esc did not start in a good state, then print the configuration issues out via the serial port, by invoking the print_configuration_issues method of the esc.
            print_configuration_issues();
            delayMicroseconds(10'000'000);
        }
        // Attempt to real serial input.
        read_input();
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    bool EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::start()
    {
#if !DISABLE_SPWM_PIN_MODIFICATION
        // Enable power circuit.
        digitalWrite(spwm_pin_config.en, HIGH);
#endif

        // Start encoder sample validator and record started status.
        started = RotaryEncoderSampleValidator::start();
        // Indicate that we have attempted to start the ESC.
        start_attempted = true;
        // If startup did work then fault and stop the SPWM output.
        if (started == false)
        {
            this->fault = true;
            stop();
        }

        // Set initial direction.
        if (com_direction_value == 0)
        {
            direction = RotationDirection::Clockwise;
            set_direction(RotaryEncoderSampleValidator::Direction::Clockwise); // update validated direction ignored if set_direction_enforcement(false)
        }
        else if (com_direction_value == 1)
        {
            direction = RotationDirection::CounterClockwise;
            set_direction(RotaryEncoderSampleValidator::Direction::CounterClockwise); // update validated direction ignored if set_direction_enforcement(false)
        }

        // Return the started status.
        return started;
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::stop()
    {
        // Indicate that we are stopped.
        start_attempted = false;
        started = false;
        // Stop the encoder sample validator.
        RotaryEncoderSampleValidator::stop();

#if !DISABLE_SPWM_PIN_MODIFICATION
        // Disable SPWM pins.
        digitalWrite(spwm_pin_config.en, LOW);
        digitalWrite(spwm_pin_config.phase_a, LOW);
        digitalWrite(spwm_pin_config.phase_b, LOW);
        digitalWrite(spwm_pin_config.phase_c, LOW);
#endif
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::log()
    {
        // Note double/float Serial.print giving 2 decimal places... use Serial.print(<float>,<decimal_places>) for more precision
        // Log ESC state data to serial port.
        cli();
        double seconds_elapsed = (double)this->micros_since_last_log * 1e-6;
        Serial.print(((double)this->loop_ctr) / seconds_elapsed);
        Serial.print(",");
        Serial.print(((double)this->sample_ctr) / seconds_elapsed);
        Serial.print(",");
        Serial.print(eular_vec_store[0], 4);

#if ENABLE_VERBOSE_LOGGING
        Serial.print(",");
        Serial.print(this->com_torque_percentage, 4);
        Serial.print(",");
        Serial.print(this->com_direction_value);
        Serial.print(",");
        Serial.print(eular_vec_store[1]);
        Serial.print(",");
        Serial.print(eular_vec_store[2]);
        Serial.print(",");
        Serial.print(eular_vec_store[3]);
        Serial.print(",");
        Serial.print(eular_vec_store[4]);
        Serial.print(",");
        Serial.print((double)kalman_vec_store[0] / (double)ENCODER_DIVISIONS);
        Serial.print(",");
        Serial.print((double)kalman_vec_store[1] / (double)ENCODER_DIVISIONS);
        Serial.print(",");
        Serial.print((double)kalman_vec_store[2] / (double)ENCODER_DIVISIONS);
        Serial.print(",");
        Serial.print((double)kalman_vec_store[3] / (double)ENCODER_DIVISIONS);
        Serial.print(",");
        Serial.print(current_triplet.phase_a);
        Serial.print(",");
        Serial.print(current_triplet.phase_b);
        Serial.print(",");
        Serial.print(current_triplet.phase_c);
        Serial.print(",");
        Serial.print(current_encoder_displacement);
#endif
        Serial.print("\n");

        // Reset loop counter and time since last log.
        this->loop_ctr = 0;
        this->sample_ctr = 0;
        this->micros_since_last_log = 0;
        sei();
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    bool EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::get_fault_status()
    {
        // Return fault state.
        return this->fault;
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    bool EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::get_started_status()
    {
        // Return started state.
        return this->started;
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::process_host_control_word(uint32_t control_word, uint32_t *data_buffer)
    {
        // Handle serial word input.
        uint16_t com_torque_value = 0;
        switch (control_word)
        {
        case SerialInputCommandWord::Null:
            break;
        case SerialInputCommandWord::Start:
            if (fault == false)
            {
                start();
            }
            break;
        case SerialInputCommandWord::Stop:
            if (fault == false)
            {
                stop();
            }
            break;
        case SerialInputCommandWord::Reset:
            stop();
            RotaryEncoderSampleValidator::reset();
            fault = false;
            break;
        case SerialInputCommandWord::Thrust1UI16:
            com_torque_value = (data_buffer[1] << 8) | data_buffer[0];
            com_torque_percentage = ((double)com_torque_value / (double)65535) * 0.5; // cap at 50%
            break;
        case SerialInputCommandWord::Direction1UI8:
            if (com_torque_percentage == 0.0) // dont reverse unless thrust is zero
            {
                com_direction_value = data_buffer[0];
                if (com_direction_value == 0)
                {
                    direction = RotationDirection::Clockwise;
                    bl_direction = false;
                    set_direction(RotaryEncoderSampleValidator::Direction::Clockwise); // update validated direction ignored if set_direction_enforcement(false)
                }
                else if (com_direction_value == 1)
                {
                    direction = RotationDirection::CounterClockwise;
                    bl_direction = true;
                    set_direction(RotaryEncoderSampleValidator::Direction::CounterClockwise); // update validated direction ignored if set_direction_enforcement(false)
                }
            }
            break;
        default:
            // unknown word
            break;
        }
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    double EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::deg_to_rad(double deg)
    {
        return deg * (M_PI / 180.0);
    };

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    double EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::rad_to_deg(double rad)
    {
        return rad * (180.0 / M_PI);
    };

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    double EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::fnmod(double value, double mod)
    {
        return value - mod * floor(value / mod);
    };

    // raw_encoder_value_to_compressed_encoder_value
    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    uint32_t EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::raw_encoder_value_to_compressed_encoder_value(double raw_encoder_value)
    {
        // compress the encoder displacement to the new range.
        double compressed_encoder_displacement_value_raw = (raw_encoder_value / encoder_compression_factor_dbl);
        // could have rounded up and therefore gone > spwm_angular_resolution_dbl, so perform mod to bring back to zero if needed.
        uint32_t compressed_encoder_value_mod = (uint32_t)round(compressed_encoder_displacement_value_raw) % spwm_angular_resolution_uint32;
        return compressed_encoder_value_mod;
    };

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    SPWMVoltageDutyTriplet EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::get_pwm_triplet(double com_torque_percentage, uint32_t encoder_current_compressed_displacement, RotationDirection direction)
    {

        double phase_a_lookup;
        double phase_b_lookup;
        double phase_c_lookup;

        phase_a_lookup = (double)this->voltage_map_ptr[this->bl_direction][0][encoder_current_compressed_displacement] / 2.0;
        phase_b_lookup = (double)this->voltage_map_ptr[this->bl_direction][1][encoder_current_compressed_displacement] / 2.0;
        phase_c_lookup = (double)this->voltage_map_ptr[this->bl_direction][2][encoder_current_compressed_displacement] / 2.0;

        // e.g. -2047 -> 2047 gets mapped to .... -1023.5 -> 1023.5
        // we then need to add half the midpoint

        //  MAX_DUTY = std::pow(2, PWM_WRITE_RESOLUTION) - 1 = 2^11 - 1 = 2048 - 1 = 2047
        // half of this is 2047/2 = 1023.5
        // check maths
        // -1023.5  + 1023.5 = 0
        // 1023.5 + 1023.5 = 2047
        // this is ok

        double half_max_duty = (double)MAX_DUTY / 2;

        SPWMVoltageDutyTriplet triplet = SPWMVoltageDutyTriplet();

        double phase_a = 0.0;
        double phase_b = 0.0;
        double phase_c = 0.0;

        double phase_a_before_correction = ((phase_a_lookup * com_torque_percentage) + half_max_duty);
        double phase_b_before_correction = ((phase_b_lookup * com_torque_percentage) + half_max_duty);
        double phase_c_before_correction = ((phase_c_lookup * com_torque_percentage) + half_max_duty);
        // find correction
        if (anti_cogging_enabled == true)
        {
            // float value = (*ptr_to_AC_MAP)[1][100];
            // double modified_duty = (double) current_duty;
            float correction = 0.0;
            if (direction == RotationDirection::Clockwise)
            {
                correction = (this->ac_map_ptr)[0][encoder_current_compressed_displacement];
            }
            else
            {
                correction = (this->ac_map_ptr)[1][encoder_current_compressed_displacement];
            }

            double phase_a_after_correction = phase_a_before_correction + (correction * 0.25);
            double phase_b_after_correction = phase_b_before_correction + (correction * 0.25);
            double phase_c_after_correction = phase_c_before_correction + (correction * 0.25);

            if ((phase_a_before_correction < half_max_duty && phase_a_after_correction > half_max_duty) || (phase_a_before_correction > half_max_duty && phase_a_after_correction < half_max_duty))
            {
                phase_a = half_max_duty;
            }
            else
            {
                phase_a = round(phase_a_after_correction);
            }

            if ((phase_b_before_correction < half_max_duty && phase_b_after_correction > half_max_duty) || (phase_b_before_correction > half_max_duty && phase_b_after_correction < half_max_duty))
            {
                phase_b = half_max_duty;
            }
            else
            {
                phase_b = round(phase_b_after_correction);
            }

            if ((phase_c_before_correction < half_max_duty && phase_c_after_correction > half_max_duty) || (phase_c_before_correction > half_max_duty && phase_c_after_correction < half_max_duty))
            {
                phase_c = half_max_duty;
            }
            else
            {
                phase_c = round(phase_c_after_correction);
            }

            // if phase_a_lookup * com_torque_percentage is above the midpoint and after correct lower than the midpoint clamp to midpoint.
            // if phase_a_lookup * com_torque_percentage is below the midpoint and after correction higher than the midpoint clamp to midpoint.

            // when clamping to zero we care about the midpoint 4096 / 2 if phase_x + midpoint

            /*phase_a = phase_a + (0.18 * correction);
            if (phase_a < 0)
            {
                phase_a = 0;
            }
            phase_b = phase_b + (0.18 * correction);
            if (phase_b < 0)
            {
                phase_b = 0;
            }
            phase_c = phase_c + (0.18 * correction);
            if (phase_c < 0)
            {
                phase_c = 0;
            }*/
        }
        else
        {
            phase_a = round(phase_a_before_correction);
            phase_b = round(phase_b_before_correction);
            phase_c = round(phase_c_before_correction);

            // check....
            // phase_a_lookup = -1023.5 * 0.5 + 1023.5 = 511.75 # -511.75 below 1023.5
            // phase_a_lookup = 0 * 0.5 + 1023.5 = 1023.5
            // phase_a_lookup = (+1023.5 * 0.5) + 1023.5 = 1535.25 # 511.75 above 1023.5
        }

        triplet.phase_a = phase_a;
        triplet.phase_b = phase_b;
        triplet.phase_c = phase_c;

        triplet.phase_a = triplet.phase_a > MAX_DUTY ? MAX_DUTY : triplet.phase_a;
        triplet.phase_b = triplet.phase_b > MAX_DUTY ? MAX_DUTY : triplet.phase_b;
        triplet.phase_c = triplet.phase_c > MAX_DUTY ? MAX_DUTY : triplet.phase_c;

        // Serial.print(triplet.phase_a); Serial.print("\n");

        return triplet;
    }
}