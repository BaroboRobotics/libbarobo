/* Filename: stand2.cpp
 * Make a MoBot stand up on a faceplate */
#include <mobot.h>

int main()
{
  CMobot mobot;

  /* Connect to the paired MoBot */
  mobot.connect();

  /* Set mobot motors to slow speed of 10 degrees per second */
  mobot.setJointSpeed(MOBOT_JOINT2, deg2rad(10));
  mobot.setJointSpeed(MOBOT_JOINT3, deg2rad(10));

  /* Set the mobot to "home" position, where all joint angles are 0 degrees. */
  mobot.moveToZero();

  /* Move the mobot into a fetal position */
  mobot.moveJointTo(MOBOT_JOINT2, deg2rad(-85));
  mobot.moveJointTo(MOBOT_JOINT3, deg2rad(80));

  /* Rotate the bottom faceplate by 45 degrees */
  mobot.moveJointTo(MOBOT_JOINT1, deg2rad(45));

  /* Lift the body up */
  mobot.moveJointTo(MOBOT_JOINT2, deg2rad(20));

  /* Pan the mobot around for 3 seconds at 90 degrees per second*/
  mobot.setJointSpeed(MOBOT_JOINT1, deg2rad(90));
  mobot.setJointSpeed(MOBOT_JOINT1, 0.30);
  mobot.moveContinuousTime( MOBOT_FORWARD,
                            MOBOT_NEUTRAL,
                            MOBOT_NEUTRAL,
                            MOBOT_NEUTRAL,
                            3000 );

  return 0;
}
