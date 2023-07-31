#include "lib/spwm_voltage_model_discretiser.cpp"
#include "lib/com.cpp"

/*
https://www.pjrc.com/teensy/td_pulse.html

bits,value     ,freq
15   0 - 32757 4577.64 Hz
14   0 - 16383 9155.27 Hz
13   0 - 8191  18310.55 Hz
12   0 - 4095  36621.09 Hz
11   0 - 2047  73242.19 Hz
10   0 - 1023  146484.38 Hz
9    0 - 511   292968.75 Hz
 */

// <std::size_t ENCODER_DIVISIONS, std::size_t ENCODER_COMPRESSION_FACTOR, std::size_t MAX_DUTY>
const std::size_t ENCODER_DIVISIONS = 16384;
const std::size_t ENCODER_VALUE_COMPRESSION = 1;
const std::size_t MAX_DUTY = 2048; //4096; //2048;

double cw_zero_displacement_deg = -1.86;
double cw_phase_displacement_deg = 240.01;
double ccw_zero_displacement_deg = 15.31;
double ccw_phase_displacement_deg = 119.99;
uint32_t number_of_poles = 14;

kaepek::SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_VALUE_COMPRESSION, MAX_DUTY> discretiser = kaepek::SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_VALUE_COMPRESSION, MAX_DUTY>(cw_zero_displacement_deg, cw_phase_displacement_deg, ccw_zero_displacement_deg, ccw_phase_displacement_deg, number_of_poles);
kaepek::SPWMVoltageDutyTriplet current_triplet;
uint32_t max_compressed_encoder_value = ENCODER_DIVISIONS / ENCODER_VALUE_COMPRESSION;
uint32_t max_duty = MAX_DUTY;
uint32_t current_simulated_encoder_displacement = 0;

void setup() {

}

void loop() {
  // get latest simulated voltages
  TORQUE_VALUE = MAX_DUTY - 1;
  kaepek::SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_VALUE_COMPRESSION, MAX_DUTY>::Direction dir = DIRECTION_VALUE == 0 ? kaepek::SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_VALUE_COMPRESSION, MAX_DUTY>::Direction::Clockwise : kaepek::SPWMVoltageModelDiscretiser<ENCODER_DIVISIONS, ENCODER_VALUE_COMPRESSION, MAX_DUTY>::Direction::CounterClockwise;
  current_triplet = discretiser.get_pwm_triplet(TORQUE_VALUE, current_simulated_encoder_displacement, dir );
  // print triplet and angle
  // Serial.print(current_simulated_encoder_displacement); Serial.print("\t");
  Serial.print(current_triplet.a); Serial.print("\t");
  Serial.print(current_triplet.b); Serial.print("\t");
  Serial.print(current_triplet.c); Serial.print("\n");
  // delay a bit
  delayMicroseconds(5000); // ~10000Hz
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
      // setup discretiser
    }

    void post_sample_logic(uint32_t encoder_value)
    {
      // need to do PWM switching here
      // generate triplet
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
  esc = kaepek::EscTeensy40AS5147P(enc, 3.0); // 3us (micro) sample period

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
    //readHostControlProfile

    
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
