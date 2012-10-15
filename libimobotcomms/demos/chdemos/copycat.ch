/* Filename: copycat.cpp
* Connects to the first two mobots in the configuration list. Sets up the
* mobots so that the second mobot copies the motions of the first mobot. */
#include <mobot.h>

CMobot mobot1;
CMobot mobot2;
CMobot mobot3;
CMobot mobot4;
CMobotGroup group1;
double angles[4];
int i;

/* Connect to the paired MoBots */
mobot1.connect();
mobot2.connect();
mobot3.connect();
mobot4.connect();

/* Add mobots to the group */
group1.addRobot(mobot2);
group1.addRobot(mobot3);
group1.addRobot(mobot4);

while(1) {
  /* Get the beginning time of loop */
  /* Get the first mobots joint angles */
  mobot1.getJointAngles(
      angles[0],
      angles[1],
      angles[2],
      angles[3]);
  /* Move the second mobot */
  group1.driveToDirectNB(
      angles[0],
      angles[1],
      angles[2],
      angles[3]);
}

