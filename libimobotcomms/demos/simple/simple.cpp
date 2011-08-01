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

  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot.poseZero();
  robot.moveWait();

  /* Rotate each of the faceplates by 90 degrees */
  robot.setMotorPosition(IMOBOT_MOTOR3, 90);
  robot.setMotorPosition(IMOBOT_MOTOR4, 90);
  /* Wait for the movement to complete */
  robot.waitMotor(IMOBOT_MOTOR3);
  robot.waitMotor(IMOBOT_MOTOR4);
  /* Move the motors back to where they were */
  robot.setMotorPosition(IMOBOT_MOTOR3, 0);
  robot.setMotorPosition(IMOBOT_MOTOR4, 0);
  robot.waitMotor(IMOBOT_MOTOR3);
  robot.waitMotor(IMOBOT_MOTOR4);

  return 0;
}
