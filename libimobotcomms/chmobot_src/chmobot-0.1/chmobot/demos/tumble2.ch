/* Filename: tumble2.ch 
 * Tumbling robot */

#include <mobot.h>
#define deg2rad(x) ((x) * M_PI/180.0)

CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

/* Move the robot into a fetal position */
robot.moveJointTo(ROBOT_JOINT2, deg2rad(-85));
robot.moveJointTo(ROBOT_JOINT3, deg2rad(80));

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
//robot.moveToZero();

/* Move the robot into a fetal position */
robot.moveJointTo(ROBOT_JOINT2, deg2rad(0));
robot.moveJointTo(ROBOT_JOINT3, deg2rad(0));
robot.moveJointTo(ROBOT_JOINT2, deg2rad(60));
robot.moveJointTo(ROBOT_JOINT3, deg2rad(-85));
robot.moveJointTo(ROBOT_JOINT2, deg2rad(80));

robot.moveJointTo(ROBOT_JOINT3, deg2rad(0));
robot.moveJointTo(ROBOT_JOINT2, deg2rad(0));
robot.moveJointTo(ROBOT_JOINT3, deg2rad(60));
robot.moveJointTo(ROBOT_JOINT2, deg2rad(-85));
robot.moveJointTo(ROBOT_JOINT3, deg2rad(80));

//robot.moveTo(0, deg2rad(70), deg2rad(-90), 0);

/* Move the robot into a fetal position */
//robot.moveJointTo(ROBOT_JOINT3, deg2rad(-85));
//robot.moveJointTo(ROBOT_JOINT2, deg2rad(80));

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
//robot.moveToZero();

/* Move the robot into a fetal position */
//robot.moveJointTo(ROBOT_JOINT2, deg2rad(-85));
//robot.moveJointTo(ROBOT_JOINT3, deg2rad(80));
