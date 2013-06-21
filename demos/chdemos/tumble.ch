/* Filename: tumble.ch 
 * Tumbling mobot */
#include <mobot.h>
CMobot robot;

/* Connect to the paired Mobot */
robot.connect();
/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.resetToZero();

/* Tumble two times */
robot.motionTumbleLeft(2);
