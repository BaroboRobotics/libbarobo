#include <stdio.h>
#include <imobotcomms.h>

CiMobotComms robot;
/* Connect to an already paired iMobot */
if(robot.connect()) {
  printf("Error connecting.\n");
}

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.poseZero();
robot.moveWait();

/* Rotate each of the faceplates by 90 degrees */
robot.setMotorPosition(IMOBOT_MOTOR3, 90);
robot.setMotorPosition(IMOBOT_MOTOR4, 90);

/* Wait for the movement to complete */
robot.waitMotor(IMOBOT_MOTOR3);
robot.waitMotor(IMOBOT_MOTOR4);
