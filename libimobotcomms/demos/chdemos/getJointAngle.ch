/* Filename: getJointAngle.ch
 * Find the current joint angle of a joint. */
#include <mobot.h>
CMobot robot;

/* Connect to a robot */
robot.connect();

/* Get the joint angle of the first joint */
double angle;
robot.getJointAngle(MOBOT_JOINT1, angle);

/* Print out the joint angle */
printf("The current joint angle for joint 1 is %lf degrees.\n", angle);
