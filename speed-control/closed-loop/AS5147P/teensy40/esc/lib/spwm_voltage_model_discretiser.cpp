#include "spwm_voltage_model_discretiser.hpp"
#include <cmath>

namespace kaepek
{
    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t MAX_DUTY>
    SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>::SPWMVoltageModelDiscretiser()
    {
        
    };

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t MAX_DUTY>
    double SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>::deg_to_rad(double deg)
    {
        return deg * (M_PI / 180.0);
    };

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t MAX_DUTY>
    double SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>::rad_to_deg(double rad)
    {
        return rad * (180.0 / M_PI);
    };

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t MAX_DUTY>
    double SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>::fnmod(double value, double mod)
    {
        return value - mod * floor(value / mod);
    };

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t MAX_DUTY>
    SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>::SPWMVoltageModelDiscretiser(double cw_zero_displacement_deg, double cw_phase_displacement_deg, double ccw_zero_displacement_deg, double ccw_phase_displacement_deg, uint32_t number_of_poles)
    {
        double cw_zero_displacement_rad = SPWMVoltageModelDiscretiser::deg_to_rad(cw_zero_displacement_deg);
        double cw_phase_displacement_rad = SPWMVoltageModelDiscretiser::deg_to_rad(cw_phase_displacement_deg);
        double ccw_zero_displacement_rad = SPWMVoltageModelDiscretiser::deg_to_rad(ccw_zero_displacement_deg);
        double ccw_phase_displacement_rad = SPWMVoltageModelDiscretiser::deg_to_rad(ccw_phase_displacement_deg);
        double sin_period_coeff = (double)number_of_poles / 2.0;

        for (uint32_t idx = 0; idx < spwm_angular_resolution_uint32; idx++)
        {
            double current_angular_position = (2.0 * (double)idx * M_PI) / spwm_angular_resolution_dbl;

            // calculate cw_phase_x_lookup.
            cw_phase_a_lookup[idx] = sin(sin_period_coeff * (current_angular_position + cw_zero_displacement_rad));
            cw_phase_b_lookup[idx] = sin(sin_period_coeff * (current_angular_position + cw_zero_displacement_rad + cw_phase_displacement_rad));
            cw_phase_c_lookup[idx] = sin(sin_period_coeff * (current_angular_position + cw_zero_displacement_rad + (2.0 * cw_phase_displacement_rad)));

            // calculate ccw_phase_x_lookup.
            ccw_phase_a_lookup[idx] = sin(sin_period_coeff * (current_angular_position + ccw_zero_displacement_rad));
            ccw_phase_b_lookup[idx] = sin(sin_period_coeff * (current_angular_position + ccw_zero_displacement_rad + ccw_phase_displacement_rad));
            ccw_phase_c_lookup[idx] = sin(sin_period_coeff * (current_angular_position + ccw_zero_displacement_rad + (2.0 * ccw_phase_displacement_rad)));
        }
    };

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t MAX_DUTY>
    SPWMVoltageDutyTriplet SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_COMPRESSION_FACTOR, MAX_DUTY>::get_pwm_triplet(uint32_t current_duty, uint32_t encoder_current_displacement, Direction direction)
    {
        // compress the encoder displacement to the new range.
        uint32_t compressed_encoder_value = round((double)encoder_current_displacement / (double)ENCODER_COMPRESSION_FACTOR);
        // could have rounded up and therefore gone > spwm_angular_resolution_dbl, so perform mod to bring back to zero if needed.
        compressed_encoder_value = (uint32_t)SPWMVoltageModelDiscretiser::fnmod((double)compressed_encoder_value, spwm_angular_resolution_dbl);
        // should check here that we have not exceeded the maximum index... todo

        double phase_a_lookup;
        double phase_b_lookup;
        double phase_c_lookup;
        if (direction == Direction::Clockwise)
        {
            // cw
            phase_a_lookup = cw_phase_a_lookup[compressed_encoder_value];
            phase_b_lookup = cw_phase_b_lookup[compressed_encoder_value];
            phase_c_lookup = cw_phase_c_lookup[compressed_encoder_value];
        }
        else
        {
            // ccw
            phase_a_lookup = ccw_phase_a_lookup[compressed_encoder_value];
            phase_b_lookup = ccw_phase_b_lookup[compressed_encoder_value];
            phase_c_lookup = ccw_phase_c_lookup[compressed_encoder_value];
        }

        SPWMVoltageDutyTriplet triplet = SPWMVoltageDutyTriplet();
        // now modify based on lookup and duty
        double current_duty_over_2 = (double) current_duty / 2.0;
        triplet.a = round((phase_a_lookup * current_duty_over_2) + current_duty_over_2);
        triplet.b = round((phase_b_lookup * current_duty_over_2) + current_duty_over_2);
        triplet.c = round((phase_c_lookup * current_duty_over_2) + current_duty_over_2);

        triplet.a = triplet.a > MAX_DUTY ? MAX_DUTY : triplet.a;
        triplet.b = triplet.b > MAX_DUTY ? MAX_DUTY : triplet.b;
        triplet.c = triplet.c > MAX_DUTY ? MAX_DUTY : triplet.c;

        return triplet;
    }
}