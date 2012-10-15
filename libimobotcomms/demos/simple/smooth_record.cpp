/* Filename: simple.cpp
 * Rotate the faceplates by 90 degrees */
#include <mobot.h>

void playBack(CMobot* mobot,
    double* time,
    double* angle1,
    double* angle2,
    double* angle3,
    double* angle4,
    int numPoints);

int main()
{
  double *time;
  double *angle1;
  double *angle2;
  double *angle3;
  double *angle4;
  int numPoints;
  CMobot mobot;

  /* Connect to the paired MoBot */
  mobot.connect();
  printf("Press any key to begin recording.\n");
  getchar();
  mobot.recordAnglesBegin(
      time,
      angle1,
      angle2,
      angle3,
      angle4,
      0.0, // period
      0);
  printf("Recording started. Press any key to stop. \n");
  getchar();
  mobot.recordAnglesEnd(numPoints);

  playBack( &mobot,
      time, 
      angle1,
      angle2,
      angle3,
      angle4,
      numPoints);

  return 0;
}

void playBack(CMobot* mobot,
    double* time,
    double* angle1,
    double* angle2,
    double* angle3,
    double* angle4,
    int numPoints)
{
  int i;
  mobot->moveTo(
      angle1[0],
      angle2[0],
      angle3[0],
      angle4[0]);
  for(i = 0; i < numPoints; i++) {
    printf("%lf %lf %lf %lf\n", 
        angle1[i],
        angle2[i],
        angle3[i],
        angle4[i]);
    mobot->driveToNB(
        angle1[i],
        angle2[i],
        angle3[i],
        angle4[i]);
  }
}
