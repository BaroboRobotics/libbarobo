/* Filename: tumble.ch 
 * Tumbling robot */
#include <mobot.h>
CMobot robot;

/* Connect to the paired Mobot */
robot.connect();
/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

/* Tumble two times */
robot.motionTumbleLeft(2);
