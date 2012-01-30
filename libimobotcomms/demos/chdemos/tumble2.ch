/* Filename: tumble2.ch 
 * Tumbling robot */

#include <mobot.h>

CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

/* Move the robot into a fetal position */
robot.moveJointTo(ROBOT_JOINT2, -85);
robot.moveJointTo(ROBOT_JOINT3, 80);

/* Begin tumbling for n times */
int n = 5;
int i;
for(i = 0; i < n; i++) {
    if((i%2) == 0) {
        robot.moveJointTo(ROBOT_JOINT2, 0);
        robot.moveJointTo(ROBOT_JOINT3, 0);
        robot.moveJointTo(ROBOT_JOINT2, 60);
        robot.moveJointTo(ROBOT_JOINT3, -85);
        robot.moveJointTo(ROBOT_JOINT2, 80);
    } else {
        robot.moveJointTo(ROBOT_JOINT3, 0);
        robot.moveJointTo(ROBOT_JOINT2, 0);
        robot.moveJointTo(ROBOT_JOINT3, 60);
        robot.moveJointTo(ROBOT_JOINT2, -85);
        robot.moveJointTo(ROBOT_JOINT3, 80);
    }
}

