#include <stdio.h>
#include <mobot.h>

#define NUM_BOTS 2
int main()
{
  CMobot robots[NUM_BOTS];
  int i;
  for(i = 0; i < NUM_BOTS; i++) {
    robots[i].connect();
  }
  CMobotGroup group;
  for(i = 0; i < NUM_BOTS; i++) {
    group.addRobot(robots[i]);
  }
  return 0;
}
