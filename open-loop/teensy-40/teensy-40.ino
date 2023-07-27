#define PIN_L6234_SPWM_PHASE_A 1
#define PIN_L6234_SPWM_PHASE_B 0
#define PIN_L6234_SPWM_PHASE_C 7
#define PIN_L6234_SPWM_EN 8
#define PWM_FREQUENCY 36000

int ELECTRICAL_DEG_PHASE_A = -1;
int ELECTRICAL_DEG_PHASE_B = 0;
int ELECTRICAL_DEG_PHASE_C = 0;
int SPWM_DUTY_PHASE_A = 0;
int SPWM_DUTY_PHASE_B = 0;
int SPWM_DUTY_PHASE_C = 0;

volatile float TORQUE_VALUE = 0.0;
volatile unsigned int DELAY_VALUE = 500;
volatile unsigned int DIRECTION_VALUE = 0;

long int t1 = millis(); // loop timing variable

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

void increment_rotation()
{
  // increment rotation
  ELECTRICAL_DEG_PHASE_A += 1;
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
  // timing diagnostics
  long int oldTime = t1;
  t1 = millis();

  // calculate SPWM duties
  SPWM_DUTY_PHASE_A = (int)(spwm_duty_calculator(ELECTRICAL_DEG_PHASE_A) * TORQUE_VALUE);
  SPWM_DUTY_PHASE_B = (int)(spwm_duty_calculator(ELECTRICAL_DEG_PHASE_B) * TORQUE_VALUE);
  SPWM_DUTY_PHASE_C = (int)(spwm_duty_calculator(ELECTRICAL_DEG_PHASE_C) * TORQUE_VALUE);

  // update PWM duties
  analogWrite(PIN_L6234_SPWM_PHASE_A, SPWM_DUTY_PHASE_A);
  analogWrite(PIN_L6234_SPWM_PHASE_B, SPWM_DUTY_PHASE_B);
  analogWrite(PIN_L6234_SPWM_PHASE_C, SPWM_DUTY_PHASE_C);

  // diagnostic prints
  Serial.print(DIRECTION_VALUE);
  Serial.print("\t");
  Serial.print(DELAY_VALUE);
  Serial.print("\t");
  Serial.print(TORQUE_VALUE);
  Serial.print("\t");
  Serial.print(t1 - oldTime);

  Serial.print("\t");
  Serial.print(SPWM_DUTY_PHASE_A);
  Serial.print("\t");
  Serial.print(SPWM_DUTY_PHASE_B);
  Serial.print("\t");
  Serial.print(SPWM_DUTY_PHASE_C);

  Serial.print("\n");

  // delay(ms)
  // delay(SPEED_POT_VALUE / 30);
  delayMicroseconds(DELAY_VALUE);
}

// we read 2 bytes in total
const int SIZE_OF_PROFILE = 4;

// buffer to store the thrust/direction profile from the serial stream
char HOST_PROFILE_BUFFER[SIZE_OF_PROFILE] = {0, 0, 0, 0};
byte HOST_PROFILE_BUFFER_CTR = 0;

bool readHostControlProfile()
{
  bool proccessedAFullProfile = false;
  cli(); // no interrupt
  while (Serial.available())
  {
    HOST_PROFILE_BUFFER[HOST_PROFILE_BUFFER_CTR] = Serial.read(); // read byte from usb
    HOST_PROFILE_BUFFER_CTR++;                                    // in buffer
    if (HOST_PROFILE_BUFFER_CTR % SIZE_OF_PROFILE == 0)
    { // when we have the right number of bytes for the whole input profile
      TORQUE_VALUE = min((float)HOST_PROFILE_BUFFER[0] / (float)255, 0.7);
      DELAY_VALUE = (HOST_PROFILE_BUFFER[2] << 8) | HOST_PROFILE_BUFFER[1];
      // extract direction from buffer (0 is cw 1 is ccw)
      DIRECTION_VALUE = HOST_PROFILE_BUFFER[3];
      proccessedAFullProfile = true; // indicate we have processed a full profile
    }
    HOST_PROFILE_BUFFER_CTR %= SIZE_OF_PROFILE; // reset buffer ctr for a new profile
  }
  sei(); // interrupt
  return proccessedAFullProfile;
}

void loop()
{
  // calculate new phase angles
  increment_rotation();

  // update pwm signal
  update_spwm_duties();

  // take user input
  readHostControlProfile();
}
