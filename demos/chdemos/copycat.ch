/* Filename: copycat.cpp
* Connects to the first two mobots in the configuration list. Sets up the
* mobots so that the second mobot copies the motions of the first mobot. */
#include <mobot.h>
#include <linkbot.h>

#define NUM_MOBOTS 4

CLinkbotI mobots[NUM_MOBOTS];

CLinkbotIGroup group1;
CLinkbotIGroup group2;
double angles[4];
int i;

/* Connect to the paired MoBots */
for(i = 0; i < NUM_MOBOTS; i++) {
  mobots[i].connect();
}

/* Add mobots to the group */
for(i = 0; i < NUM_MOBOTS/2; i++) {
  mobots[i].setColorRGB(0, 255, 0);
  mobots[i+NUM_MOBOTS/2].setColorRGB(255, 0, 0);
  mobots[i+NUM_MOBOTS/2].moveContinuousNB(ROBOT_FORWARD, ROBOT_FORWARD, ROBOT_FORWARD);
}

int j;

while(1) {
  /* Get the beginning time of loop */
  for(i = 0; i < NUM_MOBOTS/2; i++) {
    mobots[i].getJointAngles(angles[0], angles[1], angles[2]);
    for(j = 0; j < 3; j++) {
      while(angles[j] > 180) {
        angles[j] -= 360;
      } 
      while(angles[j] < -180) {
        angles[j] += 360;
      }
    }
    //mobots[i].setColorRGB(angles[0], angles[1], angles[2]);
    mobots[i+NUM_MOBOTS/2].setJointSpeeds(angles[0], angles[1]/2, -angles[2]);
    //mobots[i+NUM_MOBOTS/2].setColorRGB(angles[0], angles[1], angles[2]);
  }
}

