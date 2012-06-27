/* Filename: stand2.cpp
 * Make a MoBot stand up on a faceplate */
#include <mobot.h>

int main()
{
  CMobot robot;

  /* Connect to the paired MoBot */
  robot.connect();

  /* Set robot motors to slow speed of 10 degrees per second */
  robot.setJointSpeed(MOBOT_JOINT2, deg2rad(10));
  robot.setJointSpeed(MOBOT_JOINT3, deg2rad(10));

  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot.moveToZero();

  /* Move the robot into a fetal position */
  robot.moveJointTo(MOBOT_JOINT2, deg2rad(-85));
  robot.moveJointTo(MOBOT_JOINT3, deg2rad(80));

  /* Rotate the bottom faceplate by 45 degrees */
  robot.moveJointTo(MOBOT_JOINT1, deg2rad(45));

  /* Lift the body up */
  robot.moveJointTo(MOBOT_JOINT2, deg2rad(20));

  /* Pan the robot around for 3 seconds at 90 degrees per second*/
  robot.setJointSpeed(MOBOT_JOINT1, deg2rad(90));
  robot.setJointSpeed(MOBOT_JOINT1, 0.30);
  robot.moveContinuousTime( ROBOT_FORWARD,
                            ROBOT_NEUTRAL,
                            ROBOT_NEUTRAL,
                            ROBOT_NEUTRAL,
                            3000 );

  return 0;
}
