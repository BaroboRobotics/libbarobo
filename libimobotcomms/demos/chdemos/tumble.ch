/* Filename: tumble.ch 
 * Tumbling robot */
#include <mobot.h>
CMobot robot;

/* Connect to the paired MoBot */
robot.connect()
/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

/* Tumble five times */
robot.motionTumble(5);
