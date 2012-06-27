/* Filename: simple.cpp
 * Rotate the faceplates by 90 degrees */
#include <mobot.h>

int main()
{
  CMobot mobot;

  /* Connect to the paired MoBot */
  mobot.connect();

  //mobot.setJointSpeed(MOBOT_JOINT1, M_PI);
  //mobot.moveJointContinuousNB(MOBOT_JOINT1, MOBOT_FORWARD);
  double angle1;
  double angle2;
  double angle3;
  double angle4;
  mobot.moveToAbs(720, 0, 0, 0);
  mobot.moveToAbs(700, 0, 0, 0);
  mobot.moveToAbs(740, 0, 0, 0);
  mobot.moveToAbs(0, 0, 0, 0);
  while(1) {
    mobot.getJointAnglesAbs(
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
