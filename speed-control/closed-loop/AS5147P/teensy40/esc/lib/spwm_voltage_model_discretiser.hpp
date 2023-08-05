#include <Arduino.h>
#include "imxrt.h"
#include <math.h>

namespace kaepek
{
    struct SPWMVoltageDutyTriplet
    {
        uint32_t a;
        uint32_t b;
        uint32_t c;
    };

    template <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t MAX_DUTY>
    class SPWMVoltageModelDiscretiser
    {
    private:
        float cw_phase_a_lookup[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0};
        float cw_phase_b_lookup[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0};
        float cw_phase_c_lookup[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0};
        float ccw_phase_a_lookup[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0};
        float ccw_phase_b_lookup[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0};
        float ccw_phase_c_lookup[ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR] = {0};
        uint32_t spwm_angular_resolution_uint32 = ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR;
        double spwm_angular_resolution_dbl = (double) (ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR);
        double encoder_compression_factor_dbl = ENCODER_COMPRESSION_FACTOR;

    public:
        /**
         * SPWMVoltageModelDiscretiser default constructor.
         */
        SPWMVoltageModelDiscretiser();

        /**
         * SPWMVoltageModelDiscretiser constructor with parameters.
         *
         * @param cw_zero_displacement_deg The zero angular displacement of the cw direction mechanical encoder position vs voltage zero point.
         * @param cw_phase_displacement_deg The phase displacement of the cw direction displacement vs voltage model.
         * @param ccw_zero_displacement_deg The zero angular displacement of the ccw direction mechanical encoder position vs voltage zero point.
         * @param ccw_phase_displacement_deg The phase displacement of the ccw direction displacement vs voltage model.
         * @param number_of_poles The number of poles that this motor has. Effects the frequency of voltage cycles per mechanical cycle.
         * @return instance of SPWMVoltageModelDiscretiser class
         */
        SPWMVoltageModelDiscretiser(double cw_zero_displacement_deg, double cw_phase_displacement_deg, double ccw_zero_displacement_deg, double ccw_phase_displacement_deg, uint32_t number_of_poles);

        /**
         * raw_encoder_value_to_compressed_encoder_value default constructor.
         * @param raw_encoder_value The encoder value as read by the sensor
         * @return Returns the encoder value compressed by the ENCODER_COMPRESSION_FACTOR
         */
        uint32_t raw_encoder_value_to_compressed_encoder_value(double raw_encoder_value);

        /**
         * SPWMVoltageModelDiscretiser Direction enum class type:
         * - CounterClockwise
         * - Clockwise
         */
        enum class Direction
        {
            CounterClockwise = 0,
            Clockwise = 1
        };

        /**
         * Method to get the current phase a,b and c duty triplet.
         * @param current_duty The current duty magnitude.
         * @param encoder_current_compressed_displacement The current encoder displacement measurement value compressed by the compression factor.
         * @return a SPWMVoltageDutyTriplet struct with the 3 phase duties.
         */
        SPWMVoltageDutyTriplet get_pwm_triplet(uint32_t current_duty, uint32_t encoder_current_compressed_displacement, Direction direction);

        /**
         * Method to get radians from degrees.
         * @param deg The number of degrees.
         * @return The number of radians.
         */
        static double deg_to_rad(double deg);

        /**
         * Method to get degrees from radians.
         * @param rad The number of radians.
         * @return The number of degrees.
         */
        static double rad_to_deg(double rad);

        /**
         * Method to apply modulus in a way which obeys negative values
         * @param value The number of to perform the modulo on.
         * @param mod The base for the modulo.
         * @return The number modulo the base
         */
        static double fnmod(double value, double mod);
    };
}

/**
 * @param encoder_divisions The number of encoder divisions around a whole circle.
 * @param encoder_compression_factor The compressed number of angular divisions around a whole circle.
 */