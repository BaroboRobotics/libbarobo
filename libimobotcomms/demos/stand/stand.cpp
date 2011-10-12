#include <stdio.h>
#include <imobotcomms.h>

int main()
{
  CiMobotComms robot;

  /* Set robot motors to speed of 0.50 */
  int i;
  for(i = IMOBOT_JOINT1; i < IMOBOT_NUM_JOINTS; i++) {
    robot.setJointSpeed(i, 0.50);
  }
  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot.moveZero();
  robot.moveWait();

  /* Move the robot into a fetal position */
  robot.moveJointTo(IMOBOT_JOINT1, -85);
  robot.moveJointTo(IMOBOT_JOINT2, 80);
  robot.moveWait();

  /* Rotate the bottom faceplate by 45 degrees */
  robot.moveJointTo(IMOBOT_JOINT3, 45);
  robot.moveWait();

  /* Lift the body up */
  robot.moveJointTo(IMOBOT_JOINT1, 20);
  robot.moveWait();

  /* Pan the robot around for 3 seconds */
  robot.setJointSpeed(IMOBOT_JOINT3, 0.0);
  robot.setJointDirection(IMOBOT_JOINT3, IMOBOT_JOINT_DIR_FORWARD);
  robot.setJointSpeed(IMOBOT_JOINT3, 0.30);
#ifndef _WIN32
  sleep(3);
#else
  Sleep(3000);
#endif
  robot.setJointSpeed(IMOBOT_JOINT3, 0.0);

  return 0;
}
