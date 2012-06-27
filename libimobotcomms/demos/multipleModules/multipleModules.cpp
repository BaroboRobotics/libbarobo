#include <stdio.h>
#include <mobot.h>

#define NUM_BOTS 2
int main()
{
  CMobot mobots[NUM_BOTS];
  int i;
  for(i = 0; i < NUM_BOTS; i++) {
    mobots[i].connect();
  }
  CMobotGroup group;
  for(i = 0; i < NUM_BOTS; i++) {
    group.addMobot(mobots[i]);
  }
  return 0;
}
