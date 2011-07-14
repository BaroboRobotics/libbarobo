#include <stdio.h>
#include <imobotcomms.h>

int main()
{
  int i;
  double enc;
  CiMobotComms robot;
  /* Connect to the iMobot with bluetooth address "00:19:88:19:FB:9E" */
  if(robot.connect("00:19:88:19:FB:9E", 20)) {
    printf("Error connecting.\n");
  }

  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot.poseZero();
  sleep(2);

  /* Rotate each of the faceplates by 90 degrees */
  robot.setMotorPosition(2, 90);
  robot.setMotorPosition(3, 90);
  /* Wait for the movement to complete */
  robot.waitMotor(2);
  robot.waitMotor(3);
  /* Move the motors back to where they were */
  robot.setMotorPosition(2, 0);
  robot.setMotorPosition(3, 0);
  robot.waitMotor(2);
  robot.waitMotor(3);

  return 0;
}
