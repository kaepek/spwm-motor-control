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
   * PositionControlL6234Teensy40 a class perform open loop position control
   */
  class PositionControlL6234Teensy40 : public SerialInputControl<4>
  {
  private:
    volatile int ELECTRICAL_DEG_PHASE_A = 0;
    volatile int ELECTRICAL_DEG_PHASE_B = 0;
    volatile int ELECTRICAL_DEG_PHASE_C = 0;
    volatile int SPWM_DUTY_PHASE_A = 0;
    volatile int SPWM_DUTY_PHASE_B = 0;
    volatile int SPWM_DUTY_PHASE_C = 0;

    volatile float TORQUE_VALUE = 0.0;
    volatile signed int PHASE_CHANGE_VALUE = 0;

    volatile float total_time = 0.0;
    volatile float deltaTime;

    bool enabled = false;

    // Logging timer interval.
    const std::size_t LOGGING_MICROS = 4556;
    long int t1 = millis(); // loop timing variable

    /**
     * Method to handler control input recieved via the serial port
     * @param control_word The SerialInputCommandWord in question e.g. Start Stop Torque1UI16
     * @param data_buffer Optional data buffer. Words which have values associated with them e.g. Torque1UI16 has two bytes worth of data in the buffer.
     */
    void process_host_control_word(uint32_t control_word, uint32_t *data_buffer)
    {
      uint16_t com_torque_value = 0;
      float float_value = 0;
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
                           this->LOGGING_MICROS, true);
                           
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
      case SerialInputCommandWord::Torque1UI16:
        com_torque_value = (data_buffer[1] << 8) | data_buffer[0];       // 0 -> 65535 // we want 0 -> 1
        TORQUE_VALUE = min((float)com_torque_value / (float)65535, 0.7); // cap max to 0.7 ratio 70%.
        break;
      case SerialInputCommandWord::Offset1F32:
        *((unsigned char *)&float_value + 0) = data_buffer[0];
        *((unsigned char *)&float_value + 1) = data_buffer[1];
        *((unsigned char *)&float_value + 2) = data_buffer[2];
        *((unsigned char *)&float_value + 3) = data_buffer[3];
        PHASE_CHANGE_VALUE = (signed int)float_value / 100.0;
        break;
      default:
        Serial.println(control_word);
        break;
      }
    }

    void update_phase()
    {
      // increment rotation
      ELECTRICAL_DEG_PHASE_A = ELECTRICAL_DEG_PHASE_A + PHASE_CHANGE_VALUE;
      ELECTRICAL_DEG_PHASE_B = ELECTRICAL_DEG_PHASE_A + 120;
      ELECTRICAL_DEG_PHASE_C = ELECTRICAL_DEG_PHASE_B + 120;
      // obey modular arithmetic (loop at 360)
      ELECTRICAL_DEG_PHASE_A %= 360;
      ELECTRICAL_DEG_PHASE_B %= 360;
      ELECTRICAL_DEG_PHASE_C %= 360;
    }

    float deg_to_rad(int16_t electrical_deg_phase_x)
    {
      return (((float)electrical_deg_phase_x) * PI) / 180;
    }

    float spwm_duty_calculator(int16_t electrical_deg_phase_x)
    {
      return sin(deg_to_rad(electrical_deg_phase_x)) * 127.5 + 127.5;
    }

    float map_int_to_float(int x, int in_min, int in_max, float out_min, float out_max)
    {
      float dbl_x = (float)x;
      float dbl_in_min = (float)in_min;
      float dbl_in_max = (float)in_max;
      return (dbl_x - dbl_in_min) * (out_max - out_min) / (dbl_in_max - dbl_in_min) + out_min;
    }

    void update_spwm_duties()
    {
      if (enabled == false)
        return;
      // timing diagnostics
      long int oldTime = t1;
      t1 = millis();
      deltaTime = (t1 - oldTime);
      total_time += (float)deltaTime / (float)1000;

      // calculate SPWM duties
      SPWM_DUTY_PHASE_A = (int)(spwm_duty_calculator(ELECTRICAL_DEG_PHASE_A) * TORQUE_VALUE);
      SPWM_DUTY_PHASE_B = (int)(spwm_duty_calculator(ELECTRICAL_DEG_PHASE_B) * TORQUE_VALUE);
      SPWM_DUTY_PHASE_C = (int)(spwm_duty_calculator(ELECTRICAL_DEG_PHASE_C) * TORQUE_VALUE);

      // update PWM duties
      analogWrite(PIN_L6234_SPWM_PHASE_A, SPWM_DUTY_PHASE_A);
      analogWrite(PIN_L6234_SPWM_PHASE_B, SPWM_DUTY_PHASE_B);
      analogWrite(PIN_L6234_SPWM_PHASE_C, SPWM_DUTY_PHASE_C);

      delayMicroseconds(1000);
    }

  public:
    /**
     * PositionControlL6234Teensy40 constructor, demostrates multiple inheritance
     */
    PositionControlL6234Teensy40()
        : SerialInputControl<4>()
    {
    }

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

      analogWriteRes(8);

      analogWriteFrequency(PIN_L6234_SPWM_PHASE_A, PWM_FREQUENCY);
      analogWriteFrequency(PIN_L6234_SPWM_PHASE_B, PWM_FREQUENCY);
      analogWriteFrequency(PIN_L6234_SPWM_PHASE_C, PWM_FREQUENCY);

      update_phase();

      // delay serial read as too early and it gets junk noise data
      while (!Serial.available())
      {
        delay(100);
      }
    }

    /**
     * Method to invoke the control input read method.
     */
    void run()
    {
      // calculate new phase angles
      update_phase();

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
      Serial.print(PHASE_CHANGE_VALUE);
      Serial.print(",");
      Serial.print(ELECTRICAL_DEG_PHASE_A);
      Serial.print(",");
      Serial.print(TORQUE_VALUE);
      Serial.print(",");
      Serial.print(deltaTime, 4);

      Serial.print(",");
      Serial.print(SPWM_DUTY_PHASE_A);
      Serial.print(",");
      Serial.print(SPWM_DUTY_PHASE_B);
      Serial.print(",");
      Serial.print(SPWM_DUTY_PHASE_C);

      Serial.print(",");
      Serial.print(total_time, 4);

      Serial.print("\n");
      sei();
    }
  };
}

// Create instance.
kaepek::PositionControlL6234Teensy40 device;

void setup()
{
  device.setup();
}

void loop()
{
  device.run();
}
