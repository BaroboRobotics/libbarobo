/* Filename: simple.cpp
 * Rotate the faceplates by 90 degrees */
#include <mobot.h>

int main()
{
  CMobot robot;

  /* Connect to the paired MoBot */
  robot.connect();

  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot.moveToZero();

  /* Rotate each of the faceplates by 90 degrees */
  robot.move(90, 0, 0, 90);
  /* Move the motors back to where they were */
  robot.move(-90, 0, 0, -90);

  return 0;
}
