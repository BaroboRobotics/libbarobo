#include <imobot.h>

int main()
{
  CiMobot robot;

  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot.poseZero();
  robot.moveWait();

  /* Rotate each of the faceplates by 90 degrees */
  robot.poseJoint(2, 90);
  robot.poseJoint(3, 90);
  /* Wait for the movement to complete */
  robot.waitMotor(2);
  robot.waitMotor(3);
  /* Move the motors back to where they were */
  robot.poseJoint(2, 0);
  robot.poseJoint(3, 0);
  robot.waitMotor(2);
  robot.waitMotor(3);

  return 0;
}
