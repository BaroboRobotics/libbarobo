#include <stdio.h>
#include <imobotcomms.h>

int main()
{
  CiMobotComms robot;
#ifndef _WIN32
  /* Connect to the iMobot with bluetooth address "00:19:88:19:FB:9E" */
  if(robot.connectAddress("00:19:88:19:FB:9E", 20)) {
    printf("Error connecting.\n");
  }
#else
  /* Connect to an already paired iMobot */
  if(robot.connect()) {
    printf("Error connecting.\n");
  }
#endif

  /* Set robot motors to speed of 50 */
  int i;
  for(i = IMOBOT_MOTOR1; i < IMOBOT_NUM_MOTORS; i++) {
    robot.setMotorSpeed(i, 50);
  }
  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot.poseZero();
  robot.moveWait();

  /* Do the inchworm gait four times */
  for(i = 0; i < 4; i++) {
    robot.setMotorPosition(IMOBOT_MOTOR1, -45);
    robot.waitMotor(IMOBOT_MOTOR1);
    robot.setMotorPosition(IMOBOT_MOTOR2, 45);
    robot.waitMotor(IMOBOT_MOTOR2);
    robot.setMotorPosition(IMOBOT_MOTOR1, 0);
    robot.waitMotor(IMOBOT_MOTOR1);
    robot.setMotorPosition(IMOBOT_MOTOR2, 0);
    robot.waitMotor(IMOBOT_MOTOR2);
  }

  return 0;
}
