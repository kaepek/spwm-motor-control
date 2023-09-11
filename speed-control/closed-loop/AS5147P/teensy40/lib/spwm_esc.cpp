#include "../com/comlib.cpp"
#include "spwm_esc.hpp"
#include "../kalman_four_state/kalman_jerk.cpp"
#include "../encoder/digital_rotary_encoder.cpp"
#include "../encoder/generic/rotary_encoder_sample_validator.cpp"
#include "spwm_voltage_model_discretiser.cpp"
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
    EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::EscL6234Teensy40AS5147P() : RotaryEncoderSampleValidator(), SerialInputControl<4>()
    {
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::EscL6234Teensy40AS5147P(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWMMotorConfig motor_config, SPWML6234PinConfig spwm_pin_config, KalmanConfig kalman_config) : RotaryEncoderSampleValidator(encoder, sample_period_microseconds), SerialInputControl<4>()
    {
        this->motor_config = motor_config;
        this->spwm_pin_config = spwm_pin_config;
        this->kalman_config = kalman_config;
        this->kalman_filter = KalmanJerk1D(kalman_config.alpha, kalman_config.x_resolution_error, kalman_config.process_noise, true, (double)ENCODER_DIVISIONS);
        this->discretiser = SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::MAX_DUTY>(motor_config.cw_zero_displacement_deg, motor_config.cw_phase_displacement_deg, motor_config.ccw_zero_displacement_deg, motor_config.ccw_phase_displacement_deg, motor_config.number_of_poles);
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::post_sample_logic(uint32_t encoder_value)
    {
        sample_ctr++;

        // Take encoder value.
        this->current_encoder_displacement = encoder_value;
        // Convert to compressed.
        uint32_t compressed_encoder_value = discretiser.raw_encoder_value_to_compressed_encoder_value(encoder_value);
        // Get and apply triplet.
        current_triplet = discretiser.get_pwm_triplet(com_torque_percentage * (double)EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::MAX_DUTY, compressed_encoder_value, discretiser_direction);
        // Set pin values.
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
    void EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::setup()
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
    void EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::loop()
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
    bool EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::start()
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
            discretiser_direction = SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::MAX_DUTY>::Direction::Clockwise;
            set_direction(RotaryEncoderSampleValidator::Direction::Clockwise); // update validated direction ignored if set_direction_enforcement(false)
        }
        else if (com_direction_value == 1)
        {
            discretiser_direction = SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::MAX_DUTY>::Direction::CounterClockwise;
            set_direction(RotaryEncoderSampleValidator::Direction::CounterClockwise); // update validated direction ignored if set_direction_enforcement(false)
        }

        // Return the started status.
        return started;
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::stop()
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
    void EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::log()
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
    bool EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::get_fault_status()
    {
        // Return fault state.
        return this->fault;
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    bool EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::get_started_status()
    {
        // Return started state.
        return this->started;
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::process_host_control_word(uint32_t control_word, uint32_t *data_buffer)
    {
        // Handle serial word input.
        uint16_t com_torque_value = 0;
        float float_value = 0;
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
            com_torque_percentage = ((double)com_torque_value / (double)65535) * 0.5; // cap at 20%
            break;
        case SerialInputCommandWord::Direction1UI8:
            if (com_torque_percentage == 0.0) // dont reverse unless thrust is zero
            {
                com_direction_value = data_buffer[0];
                if (com_direction_value == 0)
                {
                    discretiser_direction = SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::MAX_DUTY>::Direction::Clockwise;
                    set_direction(RotaryEncoderSampleValidator::Direction::Clockwise); // update validated direction ignored if set_direction_enforcement(false)
                }
                else if (com_direction_value == 1)
                {
                    discretiser_direction = SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::MAX_DUTY>::Direction::CounterClockwise;
                    set_direction(RotaryEncoderSampleValidator::Direction::CounterClockwise); // update validated direction ignored if set_direction_enforcement(false)
                }
            }
            break;
        case SerialInputCommandWord::Phase1F32:
            if (started == false)
            {
                *((unsigned char *)&float_value + 0) = data_buffer[0];
                *((unsigned char *)&float_value + 1) = data_buffer[1];
                *((unsigned char *)&float_value + 2) = data_buffer[2];
                *((unsigned char *)&float_value + 3) = data_buffer[3];
                discretiser.set_cw_phase_displacement_deg(float_value);
                discretiser.update_lookup_tables();
            }
            break;
        case SerialInputCommandWord::Phase2F32:
            if (started == false)
            {
                *((unsigned char *)&float_value + 0) = data_buffer[0];
                *((unsigned char *)&float_value + 1) = data_buffer[1];
                *((unsigned char *)&float_value + 2) = data_buffer[2];
                *((unsigned char *)&float_value + 3) = data_buffer[3];
                discretiser.set_ccw_phase_displacement_deg(float_value);
                discretiser.update_lookup_tables();
            }
            break;
        case SerialInputCommandWord::Offset1F32:
            if (started == false)
            {
                *((unsigned char *)&float_value + 0) = data_buffer[0];
                *((unsigned char *)&float_value + 1) = data_buffer[1];
                *((unsigned char *)&float_value + 2) = data_buffer[2];
                *((unsigned char *)&float_value + 3) = data_buffer[3];
                discretiser.set_cw_zero_displacement_deg(float_value);
                discretiser.update_lookup_tables();
            }
            break;
        case SerialInputCommandWord::Offset2F32:
            if (started == false)
            {
                *((unsigned char *)&float_value + 0) = data_buffer[0];
                *((unsigned char *)&float_value + 1) = data_buffer[1];
                *((unsigned char *)&float_value + 2) = data_buffer[2];
                *((unsigned char *)&float_value + 3) = data_buffer[3];
                discretiser.set_ccw_zero_displacement_deg(float_value);
                discretiser.update_lookup_tables();
            }
            break;
        default:
            // unknown word
            break;
        }
    }
}