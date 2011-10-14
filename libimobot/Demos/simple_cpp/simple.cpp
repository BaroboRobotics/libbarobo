#include <imobot.h>

int main()
{
  CiMobot robot;

  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot.moveZero();
  robot.moveWait();

  /* Rotate each of the faceplates by 90 degrees */
  robot.moveJointTo(IMOBOT_MOTOR1, 90);
  robot.moveJointTo(IMOBOT_MOTOR4, 90);
  /* Wait for the movement to complete */
  robot.waitMotor(IMOBOT_MOTOR1);
  robot.waitMotor(IMOBOT_MOTOR4);
  /* Move the motors back to where they were */
  robot.moveJointTo(IMOBOT_MOTOR1, 0);
  robot.moveJointTo(IMOBOT_MOTOR4, 0);
  robot.waitMotor(IMOBOT_MOTOR1);
  robot.waitMotor(IMOBOT_MOTOR4);

  return 0;
}
