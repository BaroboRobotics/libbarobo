#include <stdio.h>
#include "imobot.h"

int main()
{
  int i;
  double enc;
  iMobot_t robot;

  /* Initialize the robot. */
  if(iMobot_init(&robot)) {
    printf("Error initializing i2c.\n");
    return -1;
  }
  printf("Initialized.\n");

  /* Pause for 2 seconds */
  sleep(2);

  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  iMobot_moveToZero(&robot);
  iMobot_moveWait(&robot);

  /* Rotate each of the faceplates by 90 degrees */
  iMobot_moveTo(&robot, 90, 0, 0, 90);
  /* Move the motors back to where they were */
  iMobot_moveTo(&robot, 0, 0, 0, 0);

  /* The following 2 lines set up the robot to listen to Bluetooth remote
   * control commands */
  iMobot_initListenerBluetooth(&robot, 20);
  iMobot_listenerMainLoop(&robot);

  return 0;
}
