#define PIN_SPEED_POT A0
#define PIN_TORQUE_POT A1

uint32_t SPEED_POT_VALUE = 0; // range 0 1023
uint32_t TORQUE_POT_VALUE = 0; // range 0 1023 (need to map to range 0 -> 1)

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
// setup input pins
  pinMode(PIN_SPEED_POT, INPUT);
  pinMode(PIN_TORQUE_POT, INPUT);
}


float map_int_to_double(int x, int in_min, int in_max, double out_min, double out_max)
{
  // float x, float in_min, float in_max, float out_min, float out_max)
  double dbl_x = (double) x;
  double dbl_in_min = (double) in_min;
  double dbl_in_max = (double) in_max;
  return (dbl_x - dbl_in_min) * (out_max - out_min) / ( (double) dbl_in_max - dbl_in_min) + out_min;
}

void loop() {
  // put your main code here, to run repeatedly:
  SPEED_POT_VALUE = analogRead(PIN_SPEED_POT);
  TORQUE_POT_VALUE = analogRead(PIN_TORQUE_POT);

  Serial.print(SPEED_POT_VALUE);
  Serial.print("\t");
  Serial.print(TORQUE_POT_VALUE);
  Serial.print("\t");
  Serial.print(map_int_to_double(TORQUE_POT_VALUE, 0, 1023, 0.0, 1.0));
  Serial.print("\n");

}
