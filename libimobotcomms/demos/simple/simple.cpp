/* Filename: simple.cpp
 * Rotate the faceplates by 90 degrees */
#include <mobot.h>

int main()
{
  CMobot robot;

  /* Connect to the paired MoBot */
  robot.connect();

  //robot.setJointSpeed(ROBOT_JOINT1, M_PI);
  //robot.moveJointContinuousNB(ROBOT_JOINT1, ROBOT_FORWARD);
  double angle1;
  double angle2;
  double angle3;
  double angle4;
  robot.moveToAbs(720, 0, 0, 0);
  robot.moveToAbs(700, 0, 0, 0);
  robot.moveToAbs(740, 0, 0, 0);
  robot.moveToAbs(0, 0, 0, 0);
  while(1) {
    robot.getJointAnglesAbs(
        angle1,
        angle2,
        angle3,
        angle4);
    printf("%lf %lf %lf %lf\n",
        angle1,
        angle2,
        angle3,
        angle4);
  }

  return 0;
}
