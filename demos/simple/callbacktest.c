#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>

#define NUMBOTS 1

void jointcb(int millis, double j1, double j2, double j3, double j4, void* data)
{
  printf("Joint event %d: %lf %lf %lf %lf\n",
      millis, j1, j2, j3, j4);
}

void accelcb(int millis, double x, double y, double z, void* data)
{
  printf("Accel event: %d: %lf %lf %lf\n",
      millis, x, y, z);
}

int main()
{
  int i = 0;
  mobot_t mobot;
  Mobot_init(&mobot);
  //Mobot_connectWithAddress(&mobot, "LQLX", 1);
  Mobot_connectWithAddress(&mobot, "S3S3", 1);
  Mobot_enableJointEventCallback(&mobot, NULL, jointcb);
  Mobot_enableAccelEventCallback(&mobot, NULL, accelcb);
  printf("Press enter to quit.\n");
  getchar();

  return 0;
}
