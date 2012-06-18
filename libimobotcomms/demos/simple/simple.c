#include <mobot.h>

mobot_t comms;
void cb(void* mobot, int button, int buttonEvent)
{
  double time;
  double angle1;
  double angle2;
  double angle3;
  double angle4;
  printf("CB: %d %d\n", button, buttonEvent);
  /* Get position information */
  Mobot_getJointAnglesTime(&comms,
    &time,
    &angle1,
    &angle2,
    &angle3,
    &angle4);
  printf("Joint Positions: %lf %lf %lf %lf\n", 
      angle1,
      angle2,
      angle3,
      angle4);
}

int main()
{
  int err;
  Mobot_init(&comms);
  //Mobot_connect(&comms);
  err = Mobot_connectWithTCP(&comms);
  if(err) {
    printf("Connect failed with error: %d\n", err);
    return 0;
  }
  printf("Connect succeeded. Move to zero.\n");
  Mobot_moveToZero(&comms);
  return 0;
}
