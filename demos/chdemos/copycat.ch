/* Filename: copycat.cpp
* Connects to the first two robots in the configuration list. Sets up the
* robots so that the second mobot copies the motions of the first mobot. */
#include <linkbot.h>

#define NUM_MOBOTS 4

CLinkbotI robots[NUM_MOBOTS];

CLinkbotIGroup group1;
CLinkbotIGroup group2;
double angles[4];
int i;

/* Connect to the paired MoBots */
for(i = 0; i < NUM_MOBOTS; i++) {
  robots[i].connect();
}

/* Add robots to the group */
for(i = 0; i < NUM_MOBOTS/2; i++) {
  robots[i].setColorRGB(0, 255, 0);
  robots[i+NUM_MOBOTS/2].setColorRGB(255, 0, 0);
  robots[i+NUM_MOBOTS/2].moveContinuousNB(ROBOT_FORWARD, ROBOT_FORWARD, ROBOT_FORWARD);
}

int j;

while(1) {
  /* Get the beginning time of loop */
  for(i = 0; i < NUM_MOBOTS/2; i++) {
    robots[i].getJointAngles(angles[0], angles[1], angles[2]);
    for(j = 0; j < 3; j++) {
      while(angles[j] > 180) {
        angles[j] -= 360;
      } 
      while(angles[j] < -180) {
        angles[j] += 360;
      }
    }
    //robots[i].setColorRGB(angles[0], angles[1], angles[2]);
    robots[i+NUM_MOBOTS/2].setJointSpeeds(angles[0], angles[1]/2, -angles[2]);
    //robots[i+NUM_MOBOTS/2].setColorRGB(angles[0], angles[1], angles[2]);
  }
}

