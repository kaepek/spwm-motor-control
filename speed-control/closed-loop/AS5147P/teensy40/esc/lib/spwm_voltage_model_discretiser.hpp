#include <Arduino.h>
#include "imxrt.h"
#include <math.h>

namespace kaepek
{
#ifndef KAEPEK_SPWM_DISCRETISER_ESC
#define KAEPEK_SPWM_DISCRETISER_ESC

    struct SPWMVoltageDutyTriplet
    {
        uint32_t phase_a;
        uint32_t phase_b;
        uint32_t phase_c;
    };

    /**
     * SPWMVoltageModelDiscretiser
     *
     * Class to discretise a 3 phase bemf voltage sine wave vs angular displacement model into a compressed lookup table so that
     * the voltages of each phase combined in a SPWMVoltageDutyTriplet can be sampled given a specific compressed encoder value via the get_pwm_triplet method.
     * A compressed encoder value can be obtained from raw_encoder_value_to_compressed_encoder_value method given a raw encoder value.
     */
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
        double spwm_angular_resolution_dbl = (double)(ENCODER_DIVISIONS / ENCODER_COMPRESSION_FACTOR);
        double encoder_compression_factor_dbl = ENCODER_COMPRESSION_FACTOR;
        uint32_t number_of_poles;
        double cw_zero_displacement_deg;
        double cw_phase_displacement_deg;
        double ccw_zero_displacement_deg;
        double ccw_phase_displacement_deg;

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
         * Method to update cw_zero_displacement_deg.
         * @param value The value to update.
         */
        void set_cw_zero_displacement_deg(float value);

        /**
         * Method to update ccw_zero_displacement_deg.
         * @param value The value to update.
         */
        void set_ccw_zero_displacement_deg(float value);

        /**
         * Method to update cw_phase_displacement_deg.
         * @param value The value to update.
         */
        void set_cw_phase_displacement_deg(float value);

        /**
         * Method to update ccw_phase_displacement_deg.
         * @param value The value to update.
         */
        void set_ccw_phase_displacement_deg(float value);

        /**
         * Method to update the trig tables when a phase or zero displacement has been changed.
         */
        void update_lookup_tables();

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
        SPWMVoltageDutyTriplet get_pwm_triplet(double current_duty, uint32_t encoder_current_compressed_displacement, Direction direction);

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
#endif
}