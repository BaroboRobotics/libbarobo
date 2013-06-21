/* Filename: start.ch 
 * Move the mobot faceplates. */
#include <mobot.h>

CMobot robot;

/* Connect to the paired Mobot */
robot.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
printf("Reset to zero...\n");
robot.resetToZero();

/* Rotate each of the faceplates by 360 degrees */
printf("move...\n");
robot.move(360, 0, 360, 360);
printf("done.\n");
