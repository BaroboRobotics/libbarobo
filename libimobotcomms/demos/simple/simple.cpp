#include <stdio.h>
#include <mobot.h>

int main()
{
  CMobot robot("00:19:88:19:FB:9E", 20);

  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  printf("Moving home...\n");
  robot.moveZero();
  robot.moveWait();
  printf("Done.");
  sleep(1);

  /* Rotate each of the faceplates by 90 degrees */
  printf("Moving joints...\n");
  robot.moveJointTo(IMOBOT_JOINT1, 375);
  robot.moveJointTo(IMOBOT_JOINT4, 375);
  /* Wait for the movement to complete */
  robot.moveJointWait(IMOBOT_JOINT1);
  robot.moveJointWait(IMOBOT_JOINT4);
  printf("Done.\n");
  sleep(1);
  /* Move the motors back to where they were */
  printf("Moving back home...\n");
  robot.moveJointTo(IMOBOT_JOINT1, 0);
  robot.moveJointTo(IMOBOT_JOINT4, 0);
  robot.moveJointWait(IMOBOT_JOINT1);
  robot.moveJointWait(IMOBOT_JOINT4);
  printf("Done.\n");
  sleep(1);

  return 0;
}
