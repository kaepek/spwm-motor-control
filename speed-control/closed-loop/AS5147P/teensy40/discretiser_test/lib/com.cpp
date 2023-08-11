#include <Arduino.h>
#include "imxrt.h"

volatile uint16_t TORQUE_VALUE = 0; // UInt16LE
volatile unsigned int DIRECTION_VALUE = 0; // UInt8

// we read 2 bytes in total
const int SIZE_OF_PROFILE = 3;

// buffer to store the thrust/direction profile from the serial stream
char HOST_PROFILE_BUFFER[SIZE_OF_PROFILE] = {0, 0, 0};
int HOST_PROFILE_BUFFER_CTR = 0;

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
      TORQUE_VALUE = (HOST_PROFILE_BUFFER[1] << 8) | HOST_PROFILE_BUFFER[0];
      // extract direction from buffer (0 is cw 1 is ccw)
      DIRECTION_VALUE = HOST_PROFILE_BUFFER[2];
      proccessedAFullProfile = true; // indicate we have processed a full profile
    }
    HOST_PROFILE_BUFFER_CTR %= SIZE_OF_PROFILE; // reset buffer ctr for a new profile
  }
  sei(); // interrupt
  return proccessedAFullProfile;
}