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
  iMobot_moveJointTo(&robot, IMOBOT_MOTOR1, 90);
  iMobot_moveJointTo(&robot, IMOBOT_MOTOR4, 90);
  /* Wait for the movement to complete */
  iMobot_waitMotor(&robot, IMOBOT_MOTOR1);
  iMobot_waitMotor(&robot, IMOBOT_MOTOR4);
  /* Move the motors back to where they were */
  iMobot_moveJointTo(&robot, IMOBOT_MOTOR1, 0);
  iMobot_moveJointTo(&robot, IMOBOT_MOTOR4, 0);
  iMobot_waitMotor(&robot, IMOBOT_MOTOR1);
  iMobot_waitMotor(&robot, IMOBOT_MOTOR4);

  /* The following 2 lines set up the robot to listen to Bluetooth remote
   * control commands */
  iMobot_initListenerBluetooth(&robot, 20);
  iMobot_listenerMainLoop(&robot);

  return 0;
}
