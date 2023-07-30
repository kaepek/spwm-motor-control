#include "lib/spwm_voltage_model_discretiser.cpp"
#include "lib/com.cpp"

// <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t MAX_DUTY>
std::size_t ENCODER_DIVISIONS = 16384;
std::size_t ENCODER_VALUE_COMPRESSION = 4;
std::size_t MAX_DUTY = 2048;

kaepek::SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_VALUE_COMPRESSION, MAX_DUTY> discretiser = kaepek::SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_VALUE_COMPRESSION, MAX_DUTY>(-1.88, 240.01, 23.84, 240.01, 14);
kaepek::SPWMVoltageDutyTriplet current_triplet;
uint32_t max_compressed_encoder_value = ENCODER_DIVISIONS / ENCODER_VALUE_COMPRESSION;
uint32_t max_duty = MAX_DUTY;
uint32_t current_simulated_encoder_displacement = 0;
uint32_t current_duty = 100;

void setup() {

}

void loop() {
  // get latest simulated voltages
  current_triplet = discretiser.get_pwm_triplet(TORQUE_VALUE, current_simulated_encoder_displacement, kaepek::SPWMVoltageModelDiscretiser::Direction::Clockwise );
  // print triplet
  Serial.print(current_triplet.a); Serial.print("\t");
  Serial.print(current_triplet.b); Serial.print("\t");
  Serial.print(current_triplet.c); Serial.print("\n");
  // delay a bit
  delayMicroseconds(100); // ~10000Hz
  current_simulated_encoder_displacement++;
  if (max_compressed_encoder_value == current_simulated_encoder_displacement) {
    current_simulated_encoder_displacement = 0;
  }
}


// reintroduce later


/*#include <Arduino.h>
#include "imxrt.h"
#include "kalman_four_state/kalman_jerk.cpp"
#include "encoder/digital_rotary_encoder.cpp"
#include "encoder/generic/rotary_encoder_sample_validator.cpp"
#include "TeensyTimerTool.h"

using namespace TeensyTimerTool;

// Create a timer to allow for periodic logging to the serial port with the rotary sensor's Kalman and Eular state.
PeriodicTimer logging_timer(GPT2);*/

/**
 * EscTeensy40AS5147P
 *
 * Class to log out the four state (displacement, velocity, acceleration and jerk) computed by the extended Kalman method for a
 * AS5147P rotary encoder on the teensy40 platform.
 */
/*
namespace kaepek
{
  class EscTeensy40AS5147P : public RotaryEncoderSampleValidator
  {
  public:
    // Default constuctor.
    EscTeensy40AS5147P() : RotaryEncoderSampleValidator()
    {
    }

    // Constructor with parameters.
    EscTeensy40AS5147P(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds) : RotaryEncoderSampleValidator(encoder, sample_period_microseconds)
    {
    }

    void post_sample_logic(uint32_t encoder_value)
    {
      // need to do PWM switching here
    }

    void post_fault_logic(RotaryEncoderSampleValidator::Fault fault_code)
    {
      // Stop logging.
      logging_timer.stop();
      // Print that a fault has occured.
      Serial.println("A fault occured. Check your encoder connection.");
    }

  };
}

// Define encoder pin config struct.
kaepek::DigitalEncoderPinsSPI enc_pins = kaepek::DigitalEncoderPinsSPI();
// Define the encoder.
kaepek::DigitalRotaryEncoderSPI enc;
// Define the encoder esc.
kaepek::EscTeensy40AS5147P esc;
// Define bool for knowing if the esc started is a good state.
bool started_ok = false;

// Method to print Kalman state via the serial port.
void print_kalman_flat(double *kalman_vec)
{
  Serial.print(kalman_vec[0]);
  Serial.print(",");
  Serial.print(kalman_vec[1]);
  Serial.print(",");
  Serial.print(kalman_vec[2]);
  Serial.print(",");
  Serial.print(kalman_vec[3]);
}

// Method to print Kalman state via the serial port with option to ignore printing the displacement.
void print_kalman_flat(double *kalman_vec, bool dont_print_disp)
{
  if (dont_print_disp == false)
  {
    Serial.print(kalman_vec[0]);
    Serial.print(",");
  }
  Serial.print(kalman_vec[1]);
  Serial.print(",");
  Serial.print(kalman_vec[2]);
  Serial.print(",");
  Serial.print(kalman_vec[3]);
}

// Method to print the Eular state via the serial port.
void print_eular_flat(double *eular_vec)
{
  Serial.print(eular_vec[0]);
  Serial.print(",");
  Serial.print(eular_vec[1]);
  Serial.print(",");
  Serial.print(eular_vec[2]);
  Serial.print(",");
  Serial.print(eular_vec[3]);
  Serial.print(",");
  Serial.print(eular_vec[4]);
}

// Rotary encoder Kalman/Eular state storage.
kaepek::Dbl4x1 kalman_vec_store = {};
kaepek::Dbl5x1 eular_vec_store = {};

// Method to print both the Eular and Kalman state of the rotary sensor from the cache.
void print_k()
{
  print_eular_flat(eular_vec_store);
  Serial.print(",");
  print_kalman_flat(kalman_vec_store, false);
  Serial.print("\n");
}

void setup()
{

  // Setup the encoder pin configuration.
  enc_pins.csn = 10;
  enc_pins.miso = 12;
  enc_pins.mosi = 11;
  enc_pins.sck = 22;

  // Initalise the encoder with giving it the pin configuration.
  enc = kaepek::DigitalRotaryEncoderSPI(enc_pins);

  // Initalise the encoder esc.
  esc = kaepek::EscTeensy40AS5147P(enc, 2.0); // 2us (micro) sample period

  // Allow skipping ahead a maximum value of 4.0, in terms of the read encoder value measurement, before a skip is detected.
  esc.set_skip_tolerance(4.0);
  // Only allow skipping ahead twice before faulting.
  esc.set_skip_threshold(2);

  // To disable direction enforcement.
  esc.set_direction_enforcement(false);

  // Run setup procedure of the esc. Note this will invoke the encoder's setup method and therefore it is unnecessary to do it explicitly on the encoder instance.
  esc.setup();

  // Start sampling.
  started_ok = esc.start();

  if (started_ok == true)
  {
    // Begin loggin the Kalman state.
    logging_timer.begin(print_k, 10'000);
  }
}

// Create a 1D four state Kalman filter: 
double alpha = 50000.0;
double angular_resolution_error = 40.0;
double process_noise = 0.000000000001;
kaepek::KalmanJerk1D filter = kaepek::KalmanJerk1D(alpha, angular_resolution_error, process_noise, true, 16384.0); // constructor with relative time and mod limit of 16384;


void loop()
{
  if (started_ok == true)
  {
    // Check the encoder has a new sample.
    if (esc.has_new_sample() == true)
    {
      // Define variables to store the sampled encoder value and the number of elapsed microseconds since the last samples retrieval.
      uint32_t encoder_value;
      uint32_t elapsed_micros_since_last_sample;
      // Fetch the stored values from the buffer.
      esc.get_sample_and_elapsed_time(encoder_value, elapsed_micros_since_last_sample);
      // Convert microseconds to seconds.
      double seconds_since_last = (double) elapsed_micros_since_last_sample * (double) 1e-6;
      // Perform one kalman step with the data.
      filter.step(seconds_since_last, encoder_value);
      // Extract state values.
      double *kalman_vec = filter.get_kalman_vector();
      double *eular_vec = filter.get_eular_vector();
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
    esc.print_configuration_issues();
    delayMicroseconds(10'000'000);
  }
}
*/