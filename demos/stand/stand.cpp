/* Filename: stand2.cpp
 * Make a MoBot stand up on a faceplate */
#include <mobot.h>

int main()
{
  CMobot mobot;

  /* Connect to the paired MoBot */
  mobot.connect();

  /* Set mobot motors to slow speed of 10 degrees per second */
  mobot.setJointSpeed(ROBOT_JOINT2, deg2rad(10));
  mobot.setJointSpeed(ROBOT_JOINT3, deg2rad(10));

  /* Set the mobot to "home" position, where all joint angles are 0 degrees. */
  mobot.moveToZero();

  /* Move the mobot into a fetal position */
  mobot.moveJointTo(ROBOT_JOINT2, deg2rad(-85));
  mobot.moveJointTo(ROBOT_JOINT3, deg2rad(80));

  /* Rotate the bottom faceplate by 45 degrees */
  mobot.moveJointTo(ROBOT_JOINT1, deg2rad(45));

  /* Lift the body up */
  mobot.moveJointTo(ROBOT_JOINT2, deg2rad(20));

  /* Pan the mobot around for 3 seconds at 90 degrees per second*/
  mobot.setJointSpeed(ROBOT_JOINT1, deg2rad(90));
  mobot.setJointSpeed(ROBOT_JOINT1, 0.30);
  mobot.moveContinuousTime( ROBOT_FORWARD,
                            ROBOT_NEUTRAL,
                            ROBOT_NEUTRAL,
                            ROBOT_NEUTRAL,
                            3000 );

  return 0;
}
