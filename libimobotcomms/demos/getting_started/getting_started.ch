/* Filename: getting_started.ch */

#include <stdio.h>
#include <imobotcomms.h>

CMobot robot;
/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveZero();
robot.moveWait();

/* Rotate each of the faceplates by 90 degrees */
robot.moveJointTo(IMOBOT_JOINT1, 90);
robot.moveJointTo(IMOBOT_JOINT4, 90);

/* Wait for the movement to complete */
robot.moveJointWait(IMOBOT_JOINT1);
robot.moveJointWait(IMOBOT_JOINT4);
