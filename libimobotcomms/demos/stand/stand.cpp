/* Filename: stand2.cpp
 * Make a MoBot stand up on a faceplate */
#include <mobot.h>

int main()
{
  CMobot robot;

  /* Connect to the paired MoBot */
  robot.connect();

  /* Set robot motors to speed of 0.50 */
  int i;
  for(i = 1; i <= 4; i++) {
    robot.setJointSpeed((mobotJointId_t)i, 0.50);
  }
  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot.moveToZero();
  robot.moveWait();

  /* Move the robot into a fetal position */
  robot.moveJointTo(MOBOT_JOINT2, -85);
  robot.moveJointTo(MOBOT_JOINT3, 80);
  robot.moveWait();

  /* Rotate the bottom faceplate by 45 degrees */
  robot.moveJointTo(MOBOT_JOINT1, 45);
  robot.moveWait();

  /* Lift the body up */
  robot.moveJointTo(MOBOT_JOINT2, 20);
  robot.moveWait();

  /* Pan the robot around for 3 seconds */
  robot.setJointSpeed(MOBOT_JOINT1, 0.30);
  robot.moveContinuousTime( MOBOT_FORWARD,
                            MOBOT_NEUTRAL,
                            MOBOT_NEUTRAL,
                            MOBOT_NEUTRAL,
                            3000 );

  return 0;
}
