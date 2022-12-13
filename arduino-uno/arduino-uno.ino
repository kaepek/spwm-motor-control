#define PIN_L6234_SPWM_PHASE_A 9
#define PIN_L6234_SPWM_PHASE_B 10
#define PIN_L6234_SPWM_PHASE_C 11
#define PIN_L6234_SPWM_EN 8
#define PIN_SPEED_POT A0
#define PIN_TORQUE_POT A1

int ELECTRICAL_DEG_PHASE_A = -1; 
int ELECTRICAL_DEG_PHASE_B = 0;
int ELECTRICAL_DEG_PHASE_C = 0;

int SPWM_DUTY_PHASE_A = 0;
int SPWM_DUTY_PHASE_B = 0;
int SPWM_DUTY_PHASE_C = 0;

int SPEED_POT_VALUE = 0; // range 0 1023
int TORQUE_POT_VALUE = 0; // range 0 1023 (need to map to range 0 -> 1)

long int t1 = millis();

void setup() {
  Serial.begin(9600);

  // setup output pins
  pinMode(PIN_L6234_SPWM_PHASE_A, OUTPUT);
  pinMode(PIN_L6234_SPWM_PHASE_B, OUTPUT);
  pinMode(PIN_L6234_SPWM_PHASE_C, OUTPUT);
  pinMode(PIN_L6234_SPWM_EN, OUTPUT);
  digitalWrite(PIN_L6234_SPWM_EN, HIGH);
  
  // setup input pins
  pinMode(PIN_SPEED_POT, INPUT);
  pinMode(PIN_TORQUE_POT, INPUT);
  
  // Set PWM frequency to 31250Hz for pin 9, 10, 11
  TCCR0B = TCCR0B & 0b11111000 | 0x03 ;
  TCCR1B = TCCR1B & 0b11111000 | 0x01;  // 9, 10
  TCCR2B = TCCR2B & 0b11111000 | 0x01;  // 11

  increment_rotation();
}

void increment_rotation() {
  // increment rotation
  ELECTRICAL_DEG_PHASE_A += 1;
  ELECTRICAL_DEG_PHASE_B = ELECTRICAL_DEG_PHASE_A + 120;
  ELECTRICAL_DEG_PHASE_C = ELECTRICAL_DEG_PHASE_B + 120;
  // obey modular arithmetic (loop at 360)
  ELECTRICAL_DEG_PHASE_A %= 360;
  ELECTRICAL_DEG_PHASE_B %= 360;
  ELECTRICAL_DEG_PHASE_C %= 360;
}

float deg_to_rad(int16_t electrical_deg_phase_x) {
  return (((float) electrical_deg_phase_x)*PI)/180;
}

float spwm_duty_calculator(int16_t electrical_deg_phase_x) {
  return sin(deg_to_rad(electrical_deg_phase_x))*127.5+127.5;
}

float map_int_to_float(int x, int in_min, int in_max, float out_min, float out_max)
{
  // float x, float in_min, float in_max, float out_min, float out_max)
  float dbl_x = (float) x;
  float dbl_in_min = (float) in_min;
  float dbl_in_max = (float) in_max;
  return (dbl_x - dbl_in_min) * (out_max - out_min) / ( dbl_in_max - dbl_in_min) + out_min;
}

void update_spwm_duties() {
  
  long int oldTime = t1;
  t1 = millis();
  

  // read pot values
  SPEED_POT_VALUE = analogRead(PIN_SPEED_POT);
  TORQUE_POT_VALUE = analogRead(PIN_TORQUE_POT);
  float TORQUE_VALUE = map_int_to_float(TORQUE_POT_VALUE, 0, 1023, 0.0, 1.0);
  // clamp torque value
  TORQUE_VALUE = min(TORQUE_VALUE, 0.7);

  // calculate SPWM duties
  SPWM_DUTY_PHASE_A = (int) (spwm_duty_calculator(ELECTRICAL_DEG_PHASE_A)*TORQUE_VALUE);
  SPWM_DUTY_PHASE_B = (int) (spwm_duty_calculator(ELECTRICAL_DEG_PHASE_B)*TORQUE_VALUE);
  SPWM_DUTY_PHASE_C = (int) (spwm_duty_calculator(ELECTRICAL_DEG_PHASE_C)*TORQUE_VALUE);

  // update PWM duties
  
  analogWrite(PIN_L6234_SPWM_PHASE_A, SPWM_DUTY_PHASE_A);
  analogWrite(PIN_L6234_SPWM_PHASE_B, SPWM_DUTY_PHASE_B);
  analogWrite(PIN_L6234_SPWM_PHASE_C, SPWM_DUTY_PHASE_C);

  //Serial.print(SPEED_POT_VALUE);
  //Serial.print("\t");
  //Serial.print(TORQUE_VALUE);
  //Serial.print("\t");
  // Serial.print(t1-oldTime);
  

  // Serial.print("\t");
  // Serial.print(SPWM_DUTY_PHASE_A);
  // Serial.print("\t");
  // Serial.print(SPWM_DUTY_PHASE_B);
  // Serial.print("\t");
  // Serial.print(SPWM_DUTY_PHASE_C);
  
  // Serial.print("\n");

  delay(SPEED_POT_VALUE/15);
}



void loop() {
  // calculate new phase angles
  increment_rotation();

  // update pwm signal
  update_spwm_duties();

}
