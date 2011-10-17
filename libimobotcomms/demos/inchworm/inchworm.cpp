#include <stdio.h>
#include <imobotcomms.h>

int main()
{
  CMobot robot;

  /* Set robot motors to speed of 0.50 */
  int i;
  for(i = IMOBOT_JOINT1; i < IMOBOT_NUM_JOINTS; i++) {
    robot.setJointSpeed(i, 0.50);
  }
  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot.moveZero();
  robot.moveWait();

  /* Do the inchworm gait four times */
  for(i = 0; i < 4; i++) {
    robot.moveJointTo(IMOBOT_JOINT2, -45);
    robot.moveJointWait(IMOBOT_JOINT1);
    robot.moveJointTo(IMOBOT_JOINT3, 45);
    robot.moveJointWait(IMOBOT_JOINT2);
    robot.moveJointTo(IMOBOT_JOINT2, 0);
    robot.moveJointWait(IMOBOT_JOINT1);
    robot.moveJointTo(IMOBOT_JOINT3, 0);
    robot.moveJointWait(IMOBOT_JOINT2);
  }

  return 0;
}
