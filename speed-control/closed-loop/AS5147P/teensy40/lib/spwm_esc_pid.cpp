#include "spwm_esc_pid.hpp"
#include "spwm_esc.cpp"

namespace kaepek
{
#ifndef ENABLE_RK4
#define ENABLE_RK4 true
#endif

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    PidEscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::PidEscL6234Teensy40AS5147P() : EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>()
    {
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    PidEscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::PidEscL6234Teensy40AS5147P(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWMMotorConfig motor_config, SPWML6234PinConfig spwm_pin_config, KalmanConfig kalman_config, PIDConfig pid_config) : EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>(encoder, sample_period_microseconds, motor_config, spwm_pin_config, kalman_config)
    {
        proportional_coefficient = pid_config.proportional;
        differential_coefficient = pid_config.differential;
        integral_coefficient = pid_config.integral;
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    double PidEscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::calculate_eular_derivative(double value, double dt, double previous_value)
    {
        return (value - previous_value) / dt;
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void PidEscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::stop()
    {
        proportional_error = 0.0;
        integral_error = 0.0;
        differential_error = 0.0;
        previous_proportional_error = 0.0;
        EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::stop();
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void PidEscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::loop()
    {

        if (EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::started == true)
        {
            // Check the encoder has a new sample.
            if (EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::has_new_sample() == true)
            {
                // Define variables to store the sampled encoder value and the number of elapsed microseconds since the last samples retrieval.
                uint32_t encoder_value;
                uint32_t elapsed_micros_since_last_sample;
                // Fetch the stored values from the buffer.
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::get_sample_and_elapsed_time(encoder_value, elapsed_micros_since_last_sample);
                // Convert microseconds to seconds.
                double seconds_since_last = (double)elapsed_micros_since_last_sample * (double)1e-6;
                // Perform one kalman step with the data.
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::kalman_filter.step(seconds_since_last, encoder_value);
                // Extract state values.
                double *kalman_vec = EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::kalman_filter.get_kalman_vector();
                double *eular_vec = EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::kalman_filter.get_eular_vector();
                // Store state in cache ready for printing.
                cli();
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::kalman_vec_store[0] = kalman_vec[0];
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::kalman_vec_store[1] = kalman_vec[1];
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::kalman_vec_store[2] = kalman_vec[2];
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::kalman_vec_store[3] = kalman_vec[3];
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::eular_vec_store[0] = eular_vec[0];
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::eular_vec_store[1] = eular_vec[1];
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::eular_vec_store[2] = eular_vec[2];
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::eular_vec_store[3] = eular_vec[3];
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::eular_vec_store[4] = eular_vec[4];

                // Calculate errors
                proportional_error = set_point - (kalman_vec[1] / (double)ENCODER_DIVISIONS);
                integral_error += proportional_error * seconds_since_last;

                // Watch out for the integral_error saturating
                if (integral_error >= 100000.0) {
                    integral_error = 0.0;
                }

#if ENABLE_RK4
                // RK4
                double k1 = calculate_eular_derivative(proportional_error, seconds_since_last, previous_proportional_error);
                double k2 = calculate_eular_derivative(proportional_error + 0.5 * k1 * seconds_since_last, seconds_since_last, previous_proportional_error);
                double k3 = calculate_eular_derivative(proportional_error + 0.5 * k2 * seconds_since_last, seconds_since_last, previous_proportional_error);
                double k4 = calculate_eular_derivative(proportional_error + k3 * seconds_since_last, seconds_since_last, previous_proportional_error);
                differential_error = (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
#else
                // Eular
                differential_error = calculate_eular_derivative(proportional_error, seconds_since_last, previous_proportional_error);
#endif

                // Calculate duty percentage
                double duty = 0.0;

                duty = proportional_coefficient * proportional_error + integral_coefficient * integral_error + differential_coefficient * differential_error;
                duty = min(duty, 0.5); // cap at 50%

                pid_duty = duty;

                previous_proportional_error = proportional_error;

                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::loop_ctr++;
                sei();
            }
        }
        else if (EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::start_attempted == true)
        {
            // If the esc did not start in a good state, then print the configuration issues out via the serial port, by invoking the print_configuration_issues method of the esc.
            EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::print_configuration_issues();
            delayMicroseconds(10'000'000);
        }
        // Attempt to real serial input.
        EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::read_input();
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void PidEscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::process_host_control_word(uint32_t control_word, uint32_t *data_buffer)
    {
        // Handle serial word input.
        uint16_t com_torque_value = 0;
        float float_value = 0;
        switch (control_word)
        {
        case SerialInputCommandWord::Null:
            break;
        case SerialInputCommandWord::Start:
            if (EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::fault == false)
            {
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::start();
            }
            break;
        case SerialInputCommandWord::Stop:
            if (EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::fault == false)
            {
                this->stop();
            }
            break;
        case SerialInputCommandWord::Reset:
            this->stop();
            EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::RotaryEncoderSampleValidator::reset();
            EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::fault = false;
            break;
        case SerialInputCommandWord::Direction1UI8:
            if (EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::com_torque_percentage == 0.0) // dont reverse unless thrust is zero
            {
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::com_direction_value = data_buffer[0];
                if (EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::com_direction_value == 0)
                {
                    EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::discretiser_direction = SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::MAX_DUTY>::Direction::Clockwise;
                    EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::set_direction(RotaryEncoderSampleValidator::Direction::Clockwise); // update validated direction ignored if set_direction_enforcement(false)
                }
                else if (EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::com_direction_value == 1)
                {
                    EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::discretiser_direction = SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::MAX_DUTY>::Direction::CounterClockwise;
                    EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::set_direction(RotaryEncoderSampleValidator::Direction::CounterClockwise); // update validated direction ignored if set_direction_enforcement(false)
                }
            }
            break;
        case SerialInputCommandWord::Phase1F32:
            if (EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::started == false)
            {
                *((unsigned char *)&float_value + 0) = data_buffer[0];
                *((unsigned char *)&float_value + 1) = data_buffer[1];
                *((unsigned char *)&float_value + 2) = data_buffer[2];
                *((unsigned char *)&float_value + 3) = data_buffer[3];
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::discretiser.set_cw_phase_displacement_deg(float_value);
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::discretiser.update_lookup_tables();
            }
            break;
        case SerialInputCommandWord::Phase2F32:
            if (EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::started == false)
            {
                *((unsigned char *)&float_value + 0) = data_buffer[0];
                *((unsigned char *)&float_value + 1) = data_buffer[1];
                *((unsigned char *)&float_value + 2) = data_buffer[2];
                *((unsigned char *)&float_value + 3) = data_buffer[3];
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::discretiser.set_ccw_phase_displacement_deg(float_value);
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::discretiser.update_lookup_tables();
            }
            break;
        case SerialInputCommandWord::Offset1F32:
            if (EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::started == false)
            {
                *((unsigned char *)&float_value + 0) = data_buffer[0];
                *((unsigned char *)&float_value + 1) = data_buffer[1];
                *((unsigned char *)&float_value + 2) = data_buffer[2];
                *((unsigned char *)&float_value + 3) = data_buffer[3];
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::discretiser.set_cw_zero_displacement_deg(float_value);
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::discretiser.update_lookup_tables();
            }
            break;
        case SerialInputCommandWord::Offset2F32:
            if (EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::started == false)
            {
                *((unsigned char *)&float_value + 0) = data_buffer[0];
                *((unsigned char *)&float_value + 1) = data_buffer[1];
                *((unsigned char *)&float_value + 2) = data_buffer[2];
                *((unsigned char *)&float_value + 3) = data_buffer[3];
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::discretiser.set_ccw_zero_displacement_deg(float_value);
                EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::discretiser.update_lookup_tables();
            }
            break;
        case SerialInputCommandWord::Thrust1UI16:
            com_torque_value = (data_buffer[1] << 8) | data_buffer[0];
            EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::com_torque_percentage = ((double)com_torque_value / (double)65535) * 0.5; // cap at 50%
            break;
        case SerialInputCommandWord::SetPointF32:
            *((unsigned char *)&float_value + 0) = data_buffer[0];
            *((unsigned char *)&float_value + 1) = data_buffer[1];
            *((unsigned char *)&float_value + 2) = data_buffer[2];
            *((unsigned char *)&float_value + 3) = data_buffer[3];
            set_point = float_value;
            break;
        case SerialInputCommandWord::ProportionalF32:
            *((unsigned char *)&float_value + 0) = data_buffer[0];
            *((unsigned char *)&float_value + 1) = data_buffer[1];
            *((unsigned char *)&float_value + 2) = data_buffer[2];
            *((unsigned char *)&float_value + 3) = data_buffer[3];
            proportional_coefficient = float_value;
            break;
        case SerialInputCommandWord::IntegralF32:
            *((unsigned char *)&float_value + 0) = data_buffer[0];
            *((unsigned char *)&float_value + 1) = data_buffer[1];
            *((unsigned char *)&float_value + 2) = data_buffer[2];
            *((unsigned char *)&float_value + 3) = data_buffer[3];
            integral_coefficient = float_value;
            break;
        case SerialInputCommandWord::DerivativeF32:
            *((unsigned char *)&float_value + 0) = data_buffer[0];
            *((unsigned char *)&float_value + 1) = data_buffer[1];
            *((unsigned char *)&float_value + 2) = data_buffer[2];
            *((unsigned char *)&float_value + 3) = data_buffer[3];
            differential_coefficient = float_value;
            break;
        default:
            // unknown word
            break;
        }
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void PidEscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::log()
    {
        // Note double/float Serial.print giving 2 decimal places... use Serial.print(<float>,<decimal_places>) for more precision
        // Log ESC state data to serial port.


        /*
        What do we reall want to print
        loop_ctr
        sample_ctr
        time
        torque percentage
        direction

        kalman_disp
        kalman_v
        kalman_a
        kalman_j

        current_triplet_a
        _b
        _c

        encoder disp

        prop error
        integral error
        derivat error

        prop coeff
        integral coeff
        derivat coeff

        */
        cli();
        double seconds_elapsed = (double)EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::micros_since_last_log * 1e-6;
        Serial.print(((double)EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::loop_ctr) / seconds_elapsed);
        Serial.print(",");
        Serial.print(((double)EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::sample_ctr) / seconds_elapsed);
        Serial.print(",");

        Serial.print(EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::eular_vec_store[0], 4);
        Serial.print(",");
        Serial.print(EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::com_torque_percentage, 4);
        Serial.print(",");
        Serial.print(EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::com_direction_value);
        Serial.print(",");

        Serial.print((double)EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::kalman_vec_store[1] / (double)ENCODER_DIVISIONS);
        Serial.print(",");

        Serial.print(EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::current_encoder_displacement);
        Serial.print(",");

        Serial.print(proportional_coefficient);
        Serial.print(",");
        Serial.print(integral_coefficient);
        Serial.print(",");
        Serial.print(differential_coefficient);

        Serial.print(",");
        Serial.print(proportional_error);
        Serial.print(",");
        Serial.print(integral_error);
        Serial.print(",");
        Serial.print(differential_error);

        Serial.print(",");
        Serial.print(pid_duty);

        Serial.print(",");
        Serial.print(set_point);

        Serial.print("\n");

        // Reset loop counter and time since last log.
        EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::loop_ctr = 0;
        EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::sample_ctr = 0;
        EscL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::micros_since_last_log = 0;
        sei();
    }
}