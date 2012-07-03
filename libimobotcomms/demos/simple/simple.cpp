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
  double *time;
  double *angles1;
  double *angles2;
  double *angles3;
  double *angles4;
  int numDataPoints;
  mobot.recordAnglesBegin(time, 
      angles1, 
      angles2, 
      angles3, 
      angles4, 
      0.1);
  mobot.moveToAbs(720, 0, 0, 0);
  mobot.moveToAbs(0, 0, 0, 0);
  mobot.recordAnglesEnd(numDataPoints);
  for(int i = 0; i < numDataPoints; i++) {
    printf("%lf\n", angles1[i]);
  }
  printf("Recorded %d data points.\n", numDataPoints);

  return 0;
}
