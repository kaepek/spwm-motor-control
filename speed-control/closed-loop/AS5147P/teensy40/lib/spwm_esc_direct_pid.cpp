#include "spwm_esc_direct_pid.hpp"
#include "spwm_esc_direct.cpp"

namespace kaepek
{
#ifndef ENABLE_RK4
#define ENABLE_RK4 true
#endif

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    PidEscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::PidEscDirectL6234Teensy40AS5147P() : EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>()
    {
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    PidEscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::PidEscDirectL6234Teensy40AS5147P(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWML6234PinConfig spwm_pin_config, KalmanConfig kalman_config, PIDConfig pid_config, const int16_t (*voltage_map_ptr)[3][ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR]) : EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>(encoder, sample_period_microseconds, spwm_pin_config, kalman_config, voltage_map_ptr)
    {
        proportional_coefficient = pid_config.proportional;
        differential_coefficient = pid_config.differential;
        integral_coefficient = pid_config.integral;
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    PidEscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::PidEscDirectL6234Teensy40AS5147P(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds, SPWML6234PinConfig spwm_pin_config, KalmanConfig kalman_config, PIDConfig pid_config, const int16_t (*voltage_map_ptr)[3][ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR], const int16_t (*ac_map_ptr)[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR]) : EscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>(encoder, sample_period_microseconds, spwm_pin_config, kalman_config, voltage_map_ptr, ac_map_ptr)
    {
        proportional_coefficient = pid_config.proportional;
        differential_coefficient = pid_config.differential;
        integral_coefficient = pid_config.integral;
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    double PidEscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::calculate_eular_derivative(double value, double dt, double previous_value)
    {
        return (value - previous_value) / dt;
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void PidEscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::stop()
    {
        proportional_error = 0.0;
        integral_error = 0.0;
        differential_error = 0.0;
        previous_proportional_error = 0.0;
        BaseEscClass::com_torque_percentage = 0.0;
        BaseEscClass::stop();
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void PidEscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::loop()
    {

        if (BaseEscClass::started == true)
        {
            // Check the encoder has a new sample.
            if (BaseEscClass::has_new_sample() == true)
            {
                // Define variables to store the sampled encoder value and the number of elapsed microseconds since the last samples retrieval.
                uint32_t encoder_value;
                uint32_t elapsed_micros_since_last_sample;
                // Fetch the stored values from the buffer.
                BaseEscClass::get_sample_and_elapsed_time(encoder_value, elapsed_micros_since_last_sample);
                // Convert microseconds to seconds.
                double seconds_since_last = (double)elapsed_micros_since_last_sample * (double)1e-6;
                // Perform one kalman step with the data.
                BaseEscClass::kalman_filter.step(seconds_since_last, encoder_value);
                // Extract state values.
                double *kalman_vec = BaseEscClass::kalman_filter.get_kalman_vector();
                double *eular_vec = BaseEscClass::kalman_filter.get_eular_vector();
                // Store state in cache ready for printing.
                cli();
                BaseEscClass::kalman_vec_store[0] = kalman_vec[0];
                BaseEscClass::kalman_vec_store[1] = kalman_vec[1];
                BaseEscClass::kalman_vec_store[2] = kalman_vec[2];
                BaseEscClass::kalman_vec_store[3] = kalman_vec[3];
                BaseEscClass::eular_vec_store[0] = eular_vec[0];
                BaseEscClass::eular_vec_store[1] = eular_vec[1];
                BaseEscClass::eular_vec_store[2] = eular_vec[2];
                BaseEscClass::eular_vec_store[3] = eular_vec[3];
                BaseEscClass::eular_vec_store[4] = eular_vec[4];

                // Calculate errors
                // set_point = cache_set_point;
                proportional_error = cache_set_point - (kalman_vec[1] / (double)ENCODER_DIVISIONS);
                integral_error += proportional_error * seconds_since_last;

                // Watch out for the integral_error saturating
                if (integral_error >= 100000.0)
                {
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

                /*double alpha = 1 / (1 + 2 * M_PI * seconds_since_last * desired_derivative_cutoff_frequency);
                derivative_error_filtered = (1 - alpha) * derivative_error_filtered + alpha * differential_error;
                differential_error = derivative_error_filtered;*/

                double omega = 2.0 * M_PI * desired_derivative_cutoff_frequency; // this
                double alpha = omega * seconds_since_last; // this

                // Apply a second-order low-pass filter
                double derivative_error_filtered = (1.0 / (alpha * alpha + 2.0 * alpha + 1.0)) * (alpha * alpha * differential_error + 2.0 * alpha * derivative_error_filtered_1 - (alpha * alpha - 2.0 * alpha + 1.0) * derivative_error_filtered_2); // this

                // Update previous filtered error values
                derivative_error_filtered_2 = derivative_error_filtered_1; // this
                derivative_error_filtered_1 = derivative_error_filtered; // this
                differential_error = derivative_error_filtered; // this

                // differential_error = - (kalman_vec[3] / (double)ENCODER_DIVISIONS);

                // differential_error = fabs(differential_error) * differential_error;

                // Calculate duty percentage
                double duty = 0.0;

                duty = (proportional_coefficient * proportional_error) + (integral_coefficient * integral_error) + (differential_coefficient * differential_error);
                // add bias term (optional default zero)
                // duty += duty_bias

                // add power law term
                if (BaseEscClass::direction == RotationDirection::Clockwise)
                {
                    if (power_law_set_point_divisor_cw != 0.0 && power_law_root_cw != 0.0)
                    {
                        duty += pow((fabs((double) cache_set_point) / (double) power_law_set_point_divisor_cw), 1.0 / (double) power_law_root_cw);
                    }
                }
                else if (BaseEscClass::direction == RotationDirection::CounterClockwise)
                {
                    if (power_law_set_point_divisor_ccw != 0.0 && power_law_root_ccw != 0.0)
                    {
                        duty += pow((fabs((double) cache_set_point) / (double) power_law_set_point_divisor_ccw), 1.0 / (double) power_law_root_ccw);
                    }
                }

                // todo make the 0.5 cap a constructor argument

                // duty = abs(duty);
                // duty = min(abs(duty), 0.5); // cap at 50%
                if (duty < 0.0)
                {
                    duty = 0.0;
                }
                duty = min(duty, 0.3);
                pid_duty = duty;

                BaseEscClass::com_torque_percentage = pid_duty;

                previous_proportional_error = proportional_error;

                BaseEscClass::loop_ctr++;
                sei();
            }
        }
        else if (BaseEscClass::start_attempted == true)
        {
            // If the esc did not start in a good state, then print the configuration issues out via the serial port, by invoking the print_configuration_issues method of the esc.
            BaseEscClass::print_configuration_issues();
            delayMicroseconds(10'000'000);
        }
        // Attempt to real serial input.
        BaseEscClass::read_input();
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void PidEscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::process_host_control_word(uint32_t control_word, uint32_t *data_buffer)
    {
        // Handle serial word input.
        uint16_t com_torque_value = 0;
        float float_value = 0;
        switch (control_word)
        {
        case SerialInputCommandWord::Null:
            break;
        case SerialInputCommandWord::Start:
            if (BaseEscClass::fault == false)
            {
                BaseEscClass::start();
            }
            break;
        case SerialInputCommandWord::Stop:
            if (BaseEscClass::fault == false)
            {
                this->stop();
            }
            break;
        case SerialInputCommandWord::Reset:
            this->stop();
            BaseEscClass::RotaryEncoderSampleValidator::reset();
            BaseEscClass::fault = false;
            break;
        case SerialInputCommandWord::Direction1UI8:
            if (BaseEscClass::com_torque_percentage == 0.0) // dont reverse unless thrust is zero
            {
                BaseEscClass::com_direction_value = data_buffer[0];
                if (BaseEscClass::com_direction_value == 0)
                {
                    BaseEscClass::direction = RotationDirection::Clockwise;
                    BaseEscClass::bl_direction = false;
                    BaseEscClass::set_direction(RotaryEncoderSampleValidator::Direction::Clockwise); // update validated direction ignored if set_direction_enforcement(false)
                }
                else if (BaseEscClass::com_direction_value == 1)
                {
                    BaseEscClass::direction = RotationDirection::CounterClockwise;
                    BaseEscClass::bl_direction = true;
                    BaseEscClass::set_direction(RotaryEncoderSampleValidator::Direction::CounterClockwise); // update validated direction ignored if set_direction_enforcement(false)
                }
            }
            break;
        case SerialInputCommandWord::Thrust1UI16:
            com_torque_value = (data_buffer[1] << 8) | data_buffer[0];
            BaseEscClass::com_torque_percentage = ((double)com_torque_value / (double)65535) * 0.5; // cap at 50%
            break;
        case SerialInputCommandWord::SetPointF32:
            *((unsigned char *)&float_value + 0) = data_buffer[0];
            *((unsigned char *)&float_value + 1) = data_buffer[1];
            *((unsigned char *)&float_value + 2) = data_buffer[2];
            *((unsigned char *)&float_value + 3) = data_buffer[3];
            cache_set_point = float_value;
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
        case SerialInputCommandWord::PowerLawSetPointDivisorCWF32:
            *((unsigned char *)&float_value + 0) = data_buffer[0];
            *((unsigned char *)&float_value + 1) = data_buffer[1];
            *((unsigned char *)&float_value + 2) = data_buffer[2];
            *((unsigned char *)&float_value + 3) = data_buffer[3];
            power_law_set_point_divisor_cw = float_value;
            break;
        case SerialInputCommandWord::PowerLawSetPointDivisorCCWF32:
            *((unsigned char *)&float_value + 0) = data_buffer[0];
            *((unsigned char *)&float_value + 1) = data_buffer[1];
            *((unsigned char *)&float_value + 2) = data_buffer[2];
            *((unsigned char *)&float_value + 3) = data_buffer[3];
            power_law_set_point_divisor_ccw = float_value;
            break;
        case SerialInputCommandWord::PowerLawRootCWF32:
            *((unsigned char *)&float_value + 0) = data_buffer[0];
            *((unsigned char *)&float_value + 1) = data_buffer[1];
            *((unsigned char *)&float_value + 2) = data_buffer[2];
            *((unsigned char *)&float_value + 3) = data_buffer[3];
            power_law_root_cw = float_value;
            break;
        case SerialInputCommandWord::PowerLawRootCCWF32:
            *((unsigned char *)&float_value + 0) = data_buffer[0];
            *((unsigned char *)&float_value + 1) = data_buffer[1];
            *((unsigned char *)&float_value + 2) = data_buffer[2];
            *((unsigned char *)&float_value + 3) = data_buffer[3];
            power_law_root_ccw = float_value;
            break;
        default:
            // unknown word
            break;
        }
    }

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t PWM_WRITE_RESOLUTION>
    void PidEscDirectL6234Teensy40AS5147P<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, PWM_WRITE_RESOLUTION>::log()
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
        double seconds_elapsed = (double)BaseEscClass::micros_since_last_log * 1e-6;
        Serial.print(((double)BaseEscClass::loop_ctr) / seconds_elapsed);
        Serial.print(",");
        Serial.print(((double)BaseEscClass::sample_ctr) / seconds_elapsed);
        Serial.print(",");

        Serial.print(BaseEscClass::eular_vec_store[0], 4);
        Serial.print(",");
        Serial.print(BaseEscClass::com_torque_percentage, 4); // put me back!

        // Serial.print(power_law_set_point_divisor_cw, 4); // value is ok
        // Serial.print(power_law_root_cw, 4); // value is ok

        /*if (BaseEscClass::direction == RotationDirection::Clockwise)
        {
            if (power_law_set_point_divisor_cw != 0.0 && power_law_root_cw != 0.0)
            {
                double help = pow((fabs((double) cache_set_point) / (double) power_law_set_point_divisor_cw), 1.0 / (double) power_law_root_cw); // / power_law_set_point_divisor_cw
                // double help = fabs((double) cache_set_point) / (double) power_law_set_point_divisor_cw;
                // double help = 1.0 / (double) power_law_root_cw;
                Serial.print(min(help, 0.3), 6);
            }
            else {
                Serial.print(-1.0);
            }
        }
        else {
            Serial.print(-2.0);
        }*/ // works ok

        Serial.print(",");
        Serial.print(BaseEscClass::com_direction_value);
        Serial.print(",");

        Serial.print((double)BaseEscClass::kalman_vec_store[1] / (double)ENCODER_DIVISIONS, 4);
        Serial.print(",");

        Serial.print((double)BaseEscClass::kalman_vec_store[2] / (double)ENCODER_DIVISIONS, 4);
        Serial.print(",");

        Serial.print((double)BaseEscClass::kalman_vec_store[3] / (double)ENCODER_DIVISIONS, 4);
        Serial.print(",");

        Serial.print(BaseEscClass::current_encoder_displacement);
        Serial.print(",");

        /*Serial.print(proportional_coefficient);
        Serial.print(",");
        Serial.print(integral_coefficient);
        Serial.print(",");
        Serial.print(differential_coefficient);
        Serial.print(",");*/

        Serial.print(proportional_error, 4);
        Serial.print(",");
        Serial.print(integral_error, 4);
        Serial.print(",");
        Serial.print(differential_error, 4);

        Serial.print(",");
        Serial.print(pid_duty, 4);

        Serial.print(",");
        Serial.print(cache_set_point * 1.0, 4);

        double half_max_duty = (double)BaseEscClass::MAX_DUTY / 2;
        Serial.print(",");
        Serial.print((double) BaseEscClass::current_triplet.phase_a - half_max_duty);
        Serial.print(",");
        Serial.print((double) BaseEscClass::current_triplet.phase_b - half_max_duty);
        Serial.print(",");
        Serial.print((double) BaseEscClass::current_triplet.phase_c - half_max_duty);

        Serial.print("\n");

        // Reset loop counter and time since last log.
        BaseEscClass::loop_ctr = 0;
        BaseEscClass::sample_ctr = 0;
        BaseEscClass::micros_since_last_log = 0;
        sei();
    }
}