#include "butterworth.hpp"

namespace kaepek
{
    template <std::size_t ORDER>
    ButterworthFilter<ORDER>::ButterworthFilter(double cutoff_frequency, double transition_bandwidth_change_threshold_percentage) : cutoff_frequency(cutoff_frequency), sample_rate(30000.0), transition_bandwidth_change_threshold_percentage(transition_bandwidth_change_threshold_percentage)
    {
        // init coefficients
        calculate_coefficients();
    }

    template <std::size_t ORDER>
    double ButterworthFilter<ORDER>::filter(double input, double current_sample_rate)
    {
        // Update the transition_bandwidth if the sample rate has changed.
        if (current_sample_rate != sample_rate)
        {
            transition_bandwidth = current_sample_rate / (2.0 * M_PI * pow(2.0, ORDER));
        }

        // Calculate the percentage change in the transition bandwidth
        double percent_change = std::abs((current_sample_rate - sample_rate) / transition_bandwidth) * 100.0;

        // Check if the change in the transition bandwidth is greater than the threshold percentage
        if (percent_change >= transition_bandwidth_change_threshold_percentage)
        {
            sample_rate = current_sample_rate;
            calculate_coefficients();
            if (sample_rate_ticker == true)
            {
                sample_rate_ticker = false;
            }
            else
            {
                sample_rate_ticker = true;
            }
        }

        // Set the initial state for the filter's first stage with the input signal.
        states[0] = input;

        // Update the filter's state using the calculated coefficients
        for (unsigned int i = 1; i <= ORDER; i++)
        {
            states[i] = coefficients[i] * input + coefficients[i - 1] * states[i - 1];
        }

        // Return the filtered output
        return states[ORDER];
    }

    template <std::size_t ORDER>
    bool ButterworthFilter<ORDER>::sample_rate_changed()
    {
        return sample_rate_ticker;
    }

    template <std::size_t ORDER>
    void ButterworthFilter<ORDER>::calculate_coefficients()
    {
        // Calculate the angular cutoff frequency in radians per second
        double angular_cutoff_frequency = 2.0 * M_PI * cutoff_frequency / sample_rate;

        // Calculate the filter coefficient parameter
        double epsilon = 1e-12; // A small epsilon value
        double filter_coefficient_parameter = 1.0 / (tan(angular_cutoff_frequency / 2.0));
        

        // Calculate the filter coefficients for each term in the filter's transfer function using the filter coefficient parameter
        for (unsigned int k = 0; k <= ORDER; k++)
        {
            // Each coefficient is determined by multiplying the corresponding binomial coefficient (n choose r)
            // and raising the filter coefficient parameter to the power of k.
            coefficients[k] = n_choose_r(ORDER, k) * pow(filter_coefficient_parameter, k);
        }
    }

    template <std::size_t ORDER>
    double ButterworthFilter<ORDER>::n_choose_r(int n, int r)
    {
        // Calculate the binomial coefficient (n choose r)

        if (r == 0)
            return 1.0;

        double result = 1.0;

        for (int i = 1; i <= r; i++)
        {
            result *= (n - i + 1) / (double)(i);
        }

        return result;
    }
}