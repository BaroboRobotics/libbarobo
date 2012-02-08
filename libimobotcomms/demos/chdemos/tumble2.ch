/* Filename: tumble2.ch 
 * Tumbling robot */
#include <mobot.h>
CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

/* Begin tumbling for n times */
int n = 2;
int i;
for(i = 0; i < n; i++) {
    robot.moveJointTo(ROBOT_JOINT2, -85);
    robot.moveJointTo(ROBOT_JOINT3, 80);
    robot.moveJointTo(ROBOT_JOINT2, 0);
    robot.moveJointTo(ROBOT_JOINT3, 0);
    robot.moveJointTo(ROBOT_JOINT2, 80);
    robot.moveJointTo(ROBOT_JOINT2, 45);
    robot.moveJointTo(ROBOT_JOINT3, -85);
    robot.moveJointTo(ROBOT_JOINT2, 80);
    robot.moveJointTo(ROBOT_JOINT3, 0);
    robot.moveJointTo(ROBOT_JOINT2, 0);
    robot.moveJointTo(ROBOT_JOINT3, 80);
    robot.moveJointTo(ROBOT_JOINT3, 45);
}

/* Unstand the robot */
robot.moveJointToNB(ROBOT_JOINT2, 0);
robot.moveJointToNB(ROBOT_JOINT3, 0);
robot.moveWait();
robot.moveToZero();
