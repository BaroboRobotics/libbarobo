#include <stdio.h>
#include <stdlib.h>
#include <mobot.h>

#define ASSERT(x, msg) \
  if(x) { \
    printf(msg); \
    exit(0); \
  }


int main(int argc, char *argv[])
{
  mobot_t mobot;
  Mobot_init(&mobot);
  if(argc != 2) {
    printf("Usage: %s <tty device>\n", argv[0]);
    exit(0);
  }
  
  if(Mobot_connectWithTTY(&mobot, argv[1])) {
    printf("Error connecting to tty device: %s\n", argv[1]);
    exit(0);
  }

  int i;
  double d;
  double x, y, z;
  for(i = 0; i < 20; i++) {
    ASSERT(Mobot_getStatus(&mobot), "Error retrieving mobot status.\n");
    ASSERT(Mobot_getJointAngle(&mobot, MOBOT_JOINT1, &d), "Error getting joint angle.\n");
    ASSERT(Mobot_getAccelerometerData(&mobot, &x, &y, &z), "Error getting accel data.\n");
    printf("Joint angle: %lf\t", d);
    printf("Accel Data: %lf %lf %lf\n", x, y, z);
  }
  printf("Test completed.\n");
  return 0; 
}
