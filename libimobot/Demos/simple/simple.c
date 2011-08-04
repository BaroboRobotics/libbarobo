#include <stdio.h>
#include "imobot.h"

int main()
{
  int i;
  double enc;
  iMobot_t robot;

  /* Initialize the robot. */
  if(BR_init(&robot)) {
    printf("Error initializing i2c.\n");
    return -1;
  }
  printf("Initialized.\n");

  /* Pause for 2 seconds */
  sleep(2);

  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  BR_poseZero(&robot);
  BR_moveWait(&robot);

  /* Rotate each of the faceplates by 90 degrees */
  BR_setMotorPosition(&robot, 2, 90);
  BR_setMotorPosition(&robot, 3, 90);
  /* Wait for the movement to complete */
  BR_waitMotor(&robot, 2);
  BR_waitMotor(&robot, 3);
  /* Move the motors back to where they were */
  BR_setMotorPosition(&robot, 2, 0);
  BR_setMotorPosition(&robot, 3, 0);
  BR_waitMotor(&robot, 2);
  BR_waitMotor(&robot, 3);

  /* The following 2 lines set up the robot to listen to Bluetooth remote
   * control commands */
  BR_initListenerBluetooth(&robot, 20);
  BR_listenerMainLoop(&robot);

  return 0;
}
