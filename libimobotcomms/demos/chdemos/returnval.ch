/* Filename: returnval.ch 
 * Rotate the faceplates by 90 degrees */

#include <mobot.h>

CMobot robot;

/* Connect to the paired MoBot */
if(robot.connect())
{
    printf("Failed to connect to the robot.\n");
    exit(0);
}
/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

/* Rotate each of the faceplates by 360 degrees */
robot.move(360, 0, 0, 360);
/* Move the motors back to where they were */
robot.move(-360, 0, 0, -360);
