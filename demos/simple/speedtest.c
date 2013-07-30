#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*M_PI/180.0)
#endif

int main()
{
  double speed;
  mobot_t comms;
  Mobot_init(&comms);
  Mobot_connect(&comms);
  Mobot_moveJointContinuousNB(&comms, 1, ROBOT_FORWARD);
  for(speed = 60; speed > -60; speed = speed - 10) {
    Mobot_setJointSpeed(&comms, 1, DEG2RAD(speed));
    //sleep(1);
  }
  return 0;
}
