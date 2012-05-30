/* Filename: copycat.cpp
 * Connects to the first two robots in the configuration list. Sets up the
 * robots so that the second robot copies the motions of the first robot. */
#include <mobot.h>

int main()
{
  CMobot robot1;
  CMobot robot2;
  double angles[4];
  int i;

  /* Connect to the paired MoBots */
  robot1.connect();
  robot2.connect();
  /* Move both robots to zero position */
  robot1.moveToZero();
  robot2.moveToZero();
  /* Relax both robots */
  robot1.stop();
  robot2.stop();

  while(1) {
    /* Get the beginning time of loop */
    /* Get the first robots joint angles */
    robot1.getJointAnglesAbs(
        angles[0],
        angles[1],
        angles[2],
        angles[3]);
    /* Move the second robot */
    robot2.moveToPIDNB(
        angles[0],
        angles[1],
        angles[2],
        angles[3]);
  }

  return 0;
}
