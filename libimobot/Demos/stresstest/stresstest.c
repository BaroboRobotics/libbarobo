#include <stdio.h>
#include <stdlib.h>
#include "imobot.h"

void terminate();
void pose(iMobot_t* robot, double* angles, unsigned char motorMask);
int main()
{
  int i;
  double pos1[4] = {64, 0, 0, 0};
  double pos2[4] = {-64, 0, 0, 0};
  FILE *logfile;
  iMobot_t robot;
  int counter = 0;
  int sleep_time = 4;

  if(iMobot_init(&robot)) {
    printf("Error initializing i2c.\n");
    return -1;
  }
  printf("Initialized.\n");
  printf("Encoder values are: 0x%X 0x%X 0x%X 0x%X\n", 
      robot.enc[0],
      robot.enc[1],
      robot.enc[2],
      robot.enc[3] );

  /* Set motor speeds and directions */
  for(i = 0; i < 4; i++) {
    iMobot_setJointDirection(&robot, i, 0);
    iMobot_setJointSpeed(&robot, i, 30);
  }

  while(1) {
    printf("Pose1...\n");
    pose(&robot, pos1, 0x0F);
    sleep(sleep_time);
    if(iMobot_isBusy(&robot)) { terminate(); }

    printf("Pose2...\n");
    pose(&robot, pos2, 0x01);
    sleep(sleep_time);
    if(iMobot_isBusy(&robot)) { terminate(); }

    printf("Pose3...\n");
    pose(&robot, pos1, 0x01);
    sleep(sleep_time);
    if(iMobot_isBusy(&robot)) { terminate(); }

    /* Rotate the robot */
    printf("Rotate1...\n");
    for(i = 1; i < 5; i++) {
      iMobot_moveJointTo(&robot, 2, (90*i)*10);
      sleep(sleep_time);
      if(iMobot_isBusy(&robot)) { terminate(); }
    }
    /* Now rotate back */
    printf("Rotate2...\n");
    for(i = 1; i < 5; i++) {
      iMobot_moveJointTo(&robot, 2, (-90*i)*10);
      sleep(sleep_time);
      if(iMobot_isBusy(&robot)) { terminate(); }
    }

    counter++;
    printf("Done loop: %d\n", counter);
  }

  return 0;
}

void terminate()
{
  printf("Possible motor failure. %s:%d\n", __FILE__, __LINE__);
  exit(0);
}

void pose(iMobot_t* robot, double* angles, unsigned char motorMask)
{
  int i;
  for(i = 0; i < 4; i++) {
    if((1<<i) & motorMask) {
      iMobot_moveJointTo(robot, i, angles[i]);
    }
  }
}
