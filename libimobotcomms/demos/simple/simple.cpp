/* Filename: simple.cpp
 * Rotate the faceplates by 90 degrees */
#include <mobot.h>

int main()
{
  CMobot robot;

  /* Connect to the paired MoBot */
if(robot.connectWithAddress("00:06:66:43:0C:DE", 1))
{
	printf("Connect error.\n");
	exit(0);
}


  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  //robot.moveToZero();

  /* Rotate each of the faceplates by 90 degrees */
  //robot.move(90*M_PI/180.0, 0, 0, 90);
  /* Move the motors back to where they were */
  //robot.move(deg2rad(-90), 0, 0, deg2rad(-90));
robot.moveJointTo(MOBOT_JOINT2, deg2rad(70));
robot.moveJointTo(MOBOT_JOINT2, deg2rad(0));

  return 0;
}
