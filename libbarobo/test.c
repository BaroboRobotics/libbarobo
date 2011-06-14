#include <stdio.h>
#include "barobo.h"

int main()
{
  int i;
  unsigned short enc[4] = {20, 20, 30, 30};
  iMobot_t robot;

  if(BR_init(&robot)) {
    printf("Error initializing i2c.\n");
    return -1;
  }
  printf("Initialized.\n");
  printf("Encoder values are: 0x%X 0x%X 0x%X 0x%X\n", 
      robot.enc[0],
      robot.enc[1],
      robot.enc[2],
      robot.enc[3] );
  getchar();

  BR_pose(&robot, enc, 0x01);

  printf("Pose command sent.\n");
  sleep(2);
  for(i = 0; i < 4; i++) {
    enc[i] = 0;
  }
  BR_pose(&robot, enc, 0x01);

  BR_initListenerBluetooth(&robot, 20);
  BR_listenerMainLoop(&robot);

  BR_terminate(&robot);

  return 0;
}
