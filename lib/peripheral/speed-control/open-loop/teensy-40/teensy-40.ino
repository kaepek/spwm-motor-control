#include "TeensyTimerTool.h"
#include "com/comlib.cpp"
#define PIN_L6234_SPWM_PHASE_A 1
#define PIN_L6234_SPWM_PHASE_B 0
#define PIN_L6234_SPWM_PHASE_C 7
#define PIN_L6234_SPWM_EN 8
#define PWM_FREQUENCY 36000
using namespace TeensyTimerTool;
PeriodicTimer logging_timer(GPT2);

namespace kaepek
{
  /**
   * EscOpenLoopL6234Teensy40 a class perform open loop position control
   */
  class EscOpenLoopL6234Teensy40 : public SerialInputControl<4>
  {
  private:
    volatile float torque_value = 0.0;
    volatile unsigned int delay_value = 500;
    volatile int electrical_deg_phase_a = -1;
    volatile int electrical_deg_phase_b = 0;
    volatile int electrical_deg_phase_c = 0;
    volatile int spwm_duty_phase_a = 0;
    volatile int spwm_duty_phase_b = 0;
    volatile int spwm_duty_phase_c = 0;
    volatile float total_time = 0.0;
    volatile float delta_time;
    bool enabled = false;
    const std::size_t logging_micros = 4556; // Logging timer interval.
    long int t1 = millis();                  // loop timing variable
    int direction = 1;

    /**
     * Method to handler control input recieved via the serial port
     * @param control_word The SerialInputCommandWord in question e.g. Start Stop Torque1UI16
     * @param data_buffer Optional data buffer. Words which have values associated with them e.g. Torque1UI16 has two bytes worth of data in the buffer.
     */
    void process_host_control_word(uint32_t control_word, uint32_t *data_buffer)
    {
      uint16_t com_torque_value = 0;
      uint16_t com_delay_value = 0;
      byte byte_direction = 0; // UInt8
      switch (control_word)
      {
      case SerialInputCommandWord::Null:
        break;
      case SerialInputCommandWord::Start:
        enabled = true;
        digitalWrite(PIN_L6234_SPWM_EN, HIGH);
        analogWrite(PIN_L6234_SPWM_PHASE_A, 0);
        analogWrite(PIN_L6234_SPWM_PHASE_B, 0);
        analogWrite(PIN_L6234_SPWM_PHASE_C, 0);
        logging_timer.begin([this]
                            { this->log_data(); },
                            this->logging_micros, true);

        break;
      case SerialInputCommandWord::Stop:
        enabled = false;
        digitalWrite(PIN_L6234_SPWM_EN, LOW);
        analogWrite(PIN_L6234_SPWM_PHASE_A, 0);
        analogWrite(PIN_L6234_SPWM_PHASE_B, 0);
        analogWrite(PIN_L6234_SPWM_PHASE_C, 0);
        logging_timer.stop();
        break;
      case SerialInputCommandWord::Reset:
        break;
      case SerialInputCommandWord::Direction1UI8:
        byte_direction = data_buffer[0];
        if (byte_direction == 0)
        {
          direction = 1;
        }
        else if (byte_direction == 1) {
          direction = -1;
        }
        break;
      case SerialInputCommandWord::Torque1UI16:
        com_torque_value = (data_buffer[1] << 8) | data_buffer[0];       // 0 -> 65535 // we want 0 -> 1
        torque_value = min((float)com_torque_value / (float)65535, 0.7); // cap max to 0.7 ratio 70%.
        break;
      case SerialInputCommandWord::Delay1UI16:
        com_delay_value = (uint16_t)(((float)((data_buffer[1] << 8) | data_buffer[0])) / 10.0);
        delay_value = max(com_delay_value, 300);
        break;
      default:
        Serial.println(control_word);
        break;
      }
    }

    /**
     * Method to increment the rotation angle.
     */
    void increment_rotation()
    {
      // increment rotation
      electrical_deg_phase_a += direction;
      electrical_deg_phase_b = electrical_deg_phase_a + 120;
      electrical_deg_phase_c = electrical_deg_phase_b + 120;
      // obey modular arithmetic (loop at 360)
      electrical_deg_phase_a %= 360;
      electrical_deg_phase_b %= 360;
      electrical_deg_phase_c %= 360;
    }

    /**
     * Method to convert degrees to radians
     * @param electrical_deg_phase_x Electrical degrees to convert to radians.
     * @return The value of the electrical_deg_phase_x in radians
     */
    float deg_to_rad(int16_t electrical_deg_phase_x)
    {
      return (((float)electrical_deg_phase_x) * PI) / 180;
    }

    /**
     * Method to calculate spwm duty given an electrical phase in degrees.
     * @param electrical_deg_phase_x Phase electrical degrees to convert to spwm duty values.
     * @return The duty value calculated from the given electrical_deg_phase_x.
     */
    float spwm_duty_calculator(int16_t electrical_deg_phase_x)
    {
      return sin(deg_to_rad(electrical_deg_phase_x)) * 127.5 + 127.5;
    }

    /**
     * Method to convert integers to floats with a given input and output min / max value.
     * @param x The integer to convert.
     * @param in_min The minimum value x can take.
     * @param in_max The maximum value x can take.
     * @param out_min The minimum value that could be returned from this method as a float.
     * @param out_max The maximum value that could be returned from this method as a float.
     * @return The converted float value given an input int x.
     */
    float map_int_to_float(int x, int in_min, int in_max, float out_min, float out_max)
    {
      float dbl_x = (float)x;
      float dbl_in_min = (float)in_min;
      float dbl_in_max = (float)in_max;
      return (dbl_x - dbl_in_min) * (out_max - out_min) / (dbl_in_max - dbl_in_min) + out_min;
    }

    /**
     * Method to update SPWM pin value duties.
     */
    void update_spwm_duties()
    {
      if (enabled == false)
        return;

      // timing diagnostics
      long int old_time = t1;
      t1 = millis();
      delta_time = (t1 - old_time);
      total_time += (float)delta_time / (float)1000;

      // calculate SPWM duties
      spwm_duty_phase_a = (int)(spwm_duty_calculator(electrical_deg_phase_a) * torque_value);
      spwm_duty_phase_b = (int)(spwm_duty_calculator(electrical_deg_phase_b) * torque_value);
      spwm_duty_phase_c = (int)(spwm_duty_calculator(electrical_deg_phase_c) * torque_value);

      // update PWM duties
      analogWrite(PIN_L6234_SPWM_PHASE_A, spwm_duty_phase_a);
      analogWrite(PIN_L6234_SPWM_PHASE_B, spwm_duty_phase_b);
      analogWrite(PIN_L6234_SPWM_PHASE_C, spwm_duty_phase_c);

      delayMicroseconds(delay_value);
    }

  public:
    /**
     * Method to setup the system.
     */
    void setup()
    {
      Serial.begin(9600);

      // setup output pins
      pinMode(PIN_L6234_SPWM_PHASE_A, OUTPUT);
      pinMode(PIN_L6234_SPWM_PHASE_B, OUTPUT);
      pinMode(PIN_L6234_SPWM_PHASE_C, OUTPUT);
      pinMode(PIN_L6234_SPWM_EN, OUTPUT);
      digitalWrite(PIN_L6234_SPWM_EN, HIGH);

      analogWriteRes(8);

      analogWriteFrequency(PIN_L6234_SPWM_PHASE_A, PWM_FREQUENCY);
      analogWriteFrequency(PIN_L6234_SPWM_PHASE_B, PWM_FREQUENCY);
      analogWriteFrequency(PIN_L6234_SPWM_PHASE_C, PWM_FREQUENCY);

      increment_rotation();

      // delay serial read as too early and it gets junk noise data
      while (!Serial.available())
      {
        delay(100);
      }
    }

    void run()
    {
      // calculate new phase angles
      increment_rotation();

      // update pwm signal
      update_spwm_duties();

      // take user input
      read_input();
    }

    /**
     * Method to log
     */
    void log_data()
    {
      cli();

      // diagnostic prints
      Serial.print(delay_value);
      Serial.print(",");
      Serial.print(torque_value);
      Serial.print(",");
      Serial.print(delta_time);

      Serial.print(",");
      Serial.print(spwm_duty_phase_a);
      Serial.print(",");
      Serial.print(spwm_duty_phase_b);
      Serial.print(",");
      Serial.print(spwm_duty_phase_c);

      Serial.print(",");
      Serial.print(total_time, 4);

      Serial.print(",");
      Serial.print(direction);

      Serial.print("\n");

      sei();
    }
  };
}

// Create instance.
kaepek::EscOpenLoopL6234Teensy40 device;

void setup()
{
  device.setup();
}

void loop()
{
  device.run();
}
