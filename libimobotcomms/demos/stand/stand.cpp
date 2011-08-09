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

  /* Pan the robot around */
  robot.setMotorPosition(IMOBOT_MOTOR3, 90);
  robot.waitMotor(IMOBOT_MOTOR3);
  robot.setMotorPosition(IMOBOT_MOTOR3, 180);
  robot.waitMotor(IMOBOT_MOTOR3);
  robot.setMotorPosition(IMOBOT_MOTOR3, 270);
  robot.waitMotor(IMOBOT_MOTOR3);
  robot.setMotorPosition(IMOBOT_MOTOR3, 0);
  robot.waitMotor(IMOBOT_MOTOR3);

  return 0;
}
