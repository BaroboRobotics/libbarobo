/* Filename: tumble2.ch 
 * Tumbling robot */
#include <mobot.h>
CMobot robot;

/* Connect to the paired Mobot */
robot.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

/* Begin tumbling for "num" times */
int i, num = 2;
for(i = 0; i < num; i++) {
    /* First lift and tumble */
    robot.moveJointTo(ROBOT_JOINT2, -85);
    robot.moveJointTo(ROBOT_JOINT3, 80);
    robot.moveJointTo(ROBOT_JOINT2, 0);
    robot.moveJointTo(ROBOT_JOINT3, 0);
    robot.moveJointTo(ROBOT_JOINT2, 80);
    robot.moveJointTo(ROBOT_JOINT2, 45);
    /* Second lift and tumble */
    robot.moveJointTo(ROBOT_JOINT3, -85);
    robot.moveJointTo(ROBOT_JOINT2, 80);
    robot.moveJointTo(ROBOT_JOINT3, 0);
    robot.moveJointTo(ROBOT_JOINT2, 0);
    robot.moveJointTo(ROBOT_JOINT3, 80);
    if(i != (num-1)) {
        robot.moveJointTo(ROBOT_JOINT3, 45);
    }
}

/* Unstand the robot */
robot.moveJointToNB(ROBOT_JOINT2, 0);
robot.moveJointToNB(ROBOT_JOINT3, 0);
robot.moveWait();
robot.moveToZero();
