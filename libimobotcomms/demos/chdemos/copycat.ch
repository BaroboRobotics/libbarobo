/* Filename: copycat.cpp
* Connects to the first two mobots in the configuration list. Sets up the
* mobots so that the second mobot copies the motions of the first mobot. */
#include <mobot.h>

CMobot mobot1;
CMobot mobot2;
double angles[4];
int i;

/* Connect to the paired MoBots */
mobot1.connect();
mobot2.connect();
/* Move both mobots to zero position */
mobot1.moveToZero();
mobot2.moveToZero();
/* Relax both mobots */
mobot1.stopAllJoints();
mobot2.stopAllJoints();

while(1) {
  /* Get the beginning time of loop */
  /* Get the first mobots joint angles */
  mobot1.getJointAnglesAbs(
      angles[0],
      angles[1],
      angles[2],
      angles[3]);
  /* Move the second mobot */
  mobot2.moveToPIDNB(
      angles[0],
      angles[1],
      angles[2],
      angles[3]);
}

