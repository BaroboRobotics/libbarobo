#include <imobot.h>

CiMobot robot;

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.poseZero();
robot.moveWait();

/* Rotate each of the faceplates by 90 degrees */
robot.poseJoint(IMOBOT_MOTOR1, 90);
robot.poseJoint(IMOBOT_MOTOR4, 90);
/* Wait for the movement to complete */
robot.waitMotor(IMOBOT_MOTOR1);
robot.waitMotor(IMOBOT_MOTOR4);
/* Move the motors back to where they were */
robot.poseJoint(IMOBOT_MOTOR1, 0);
robot.poseJoint(IMOBOT_MOTOR4, 0);
robot.waitMotor(IMOBOT_MOTOR1);
robot.waitMotor(IMOBOT_MOTOR4);
