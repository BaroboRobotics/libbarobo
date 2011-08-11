#include <stdio.h>
#include <imobotcomms.h>

CiMobotComms robot;

/* Connect to an already paired iMobot */
if(robot.connect()) {
  printf("Error connecting.\n");
}

/* Set robot motors to speed of 50 */
int i;
for(i = IMOBOT_MOTOR1; i < IMOBOT_NUM_MOTORS; i++) {
  robot.setMotorSpeed(i, 50);
}
/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.poseZero();
robot.moveWait();

/* Move the robot into a fetal position */
robot.setMotorPosition(IMOBOT_MOTOR1, -85);
robot.setMotorPosition(IMOBOT_MOTOR2, 80);
robot.moveWait();

/* Rotate the bottom faceplate by 45 degrees */
robot.setMotorPosition(IMOBOT_MOTOR3, 45);
robot.moveWait();

/* Lift the body up */
robot.setMotorPosition(IMOBOT_MOTOR1, 20);
robot.moveWait();

/* Pan the robot around for 3 seconds */
robot.setMotorSpeed(IMOBOT_MOTOR3, 0);
robot.setMotorDirection(IMOBOT_MOTOR3, IMOBOT_MOTOR_DIR_FORWARD);
robot.setMotorSpeed(IMOBOT_MOTOR3, 30);
sleep(3);
robot.setMotorSpeed(IMOBOT_MOTOR3, 0);

