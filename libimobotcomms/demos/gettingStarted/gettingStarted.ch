/* Filename: gettingStarted.ch 
 * Move the robot endplates. */

#include <mobot.h>

CMobot robot;
double angle1, angle4; // Angles for joints 1 and 4

/* Connect to the paired MoBot */
robot.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

/* Rotate each of the faceplates by 90 degrees */
angle1 = deg2rad(90);
angle4 = deg2rad(90);
robot.move(angle1, 0, 0, angle4);
robot.moveWait();
