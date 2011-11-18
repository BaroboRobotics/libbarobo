#include <imobot.h>

int main()
{
  CiMobot robot;

  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot.moveToZero();
  robot.moveWait();

  /* Rotate each of the faceplates by 90 degrees */
  robot.moveTo(90, 0, 0, 90);
  /* Move the motors back to where they were */
  robot.moveTo(0, 0, 0, 0);

  return 0;
}
