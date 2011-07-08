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
  for(i = 0; i < 4; i++) {
    enc = 0;
    BR_poseJoint(&robot, i, enc);
  }

  /* Rotate each of the faceplates by 90 degrees */
  BR_poseJoint(&robot, 2, 90);
  BR_poseJoint(&robot, 3, 90);
  /* Wait for the movement to complete */
  BR_waitMotor(&robot, 2);
  BR_waitMotor(&robot, 3);
  /* Move the motors back to where they were */
  BR_poseJoint(&robot, 2, 0);
  BR_poseJoint(&robot, 3, 0);
  BR_waitMotor(&robot, 2);
  BR_waitMotor(&robot, 3);

  /* The following 2 lines set up the robot to listen to Bluetooth remote
   * control commands */
  BR_initListenerBluetooth(&robot, 20);
  BR_listenerMainLoop(&robot);

  /* Terminate control of the robot */
  BR_terminate(&robot);

  return 0;
}
