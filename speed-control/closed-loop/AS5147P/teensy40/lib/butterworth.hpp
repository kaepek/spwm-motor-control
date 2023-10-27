#include <cmath>

#ifndef KAEPEK_BUTTERWORTH_FILTER
#define KAEPEK_BUTTERWORTH_FILTER

namespace kaepek
{
    /**
     * ButterworthFilter - A generic nth-order Butterworth filter.
     *
     * This class implements a generic nth-order Butterworth filter with dynamic
     * adjustment of filter parameters based on changes in sample rate. The filter
     * design adapts to changes in sample rate while maintaining desired transition
     * characteristics.
     */
    template <int Order>
    class ButterworthFilter
    {
    public:
        /**
         * Constructor for the Butterworth filter.
         * @param cutoff_frequency The desired cutoff frequency for the filter.
         */
        ButterworthFilter(double cutoff_frequency);

        /**
         * Method to filter the input signal using the Butterworth filter.
         * @param input The input signal to filter.
         * @param current_sample_rate The current sample rate of the system.
         * @return The filtered output signal.
         */
        double filter(double input, double current_sample_rate);

        /**
         * Method to indicate sample rate has changed, oscillates true and false.
         */
        bool sample_rate_changed();

    private:
        double cutoff_frequency;
        double sample_rate;
        double coefficients[Order + 1] = {0.0};
        double states[Order + 1] = {0.0};
        double transition_bandwidth = 0.0;
        const double transition_bandwidth_change_threshold_percentage = 5.0; // 5%
        bool sample_rate_ticker = false;

        /**
         * Method to calculate the filter coefficients based on the current parameters.
         */
        void calculate_coefficients();

        /**
         * Method to calculate the binomial coefficient (n choose r).
         * @param n The total number of items.
         * @param r The number of items to choose.
         * @return The binomial coefficient.
         */
        double n_choose_r(int n, int r);
    };
}

#endif // KAEPEK_BUTTERWORTH_FILTER