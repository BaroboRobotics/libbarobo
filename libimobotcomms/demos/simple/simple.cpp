#include <mobot.h>

int main()
{
  CMobot robot("00:19:88:19:FB:9E", 20);

  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot.moveToZero();
  robot.moveWait();

  /* Rotate each of the faceplates by 90 degrees */
  robot.moveJointTo(MOBOT_JOINT1, 90);
  robot.moveJointTo(MOBOT_JOINT4, 90);
  /* Wait for the movement to complete */
  robot.moveJointWait(MOBOT_JOINT1);
  robot.moveJointWait(MOBOT_JOINT4);
  /* Move the motors back to where they were */
  robot.moveJointTo(MOBOT_JOINT1, 0);
  robot.moveJointTo(MOBOT_JOINT4, 0);
  robot.moveJointWait(MOBOT_JOINT1);
  robot.moveJointWait(MOBOT_JOINT4);

  return 0;
}
