/* Filename: simple.ch 
 * Rotate the faceplates by 90 degrees */

#include <mobot.h>
#define deg2rad(x) ((x) * M_PI/180.0)

CMobot robot;

/* Connect to the paired MoBot */
if(robot.connectWithAddress("00:06:66:43:0C:DE", 1))
{
	printf("Connect error.\n");
	exit(0);
}
//robot.connect();
/* Set the robot to "home" position, where all joint angles are 0 degrees. */
//robot.moveToZero();

/* Rotate each of the faceplates by 90 degrees */
//robot.move(deg2rad(90), 0, 0, deg2rad(90));
//robot.move(0, deg2rad(70), 0, 0 );
robot.moveJointTo(MOBOT_JOINT2, deg2rad(70));
robot.moveJointTo(MOBOT_JOINT2, deg2rad(0));
robot.moveJointTo(MOBOT_JOINT2, deg2rad(70));
/* Move the motors back to where they were */
//robot.move(deg2rad(-90), 0, 0, deg2rad(-90));
