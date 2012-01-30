/* File: nonblock.ch
   use the non-blocking functoin move() . */
#include <mobot.h>

CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

robot.moveToZero();

/* Rotate each of the faceplates by 720 degrees */
//robot.move(720, 0, 0, 720); // Blocking version
robot.moveNB(720, 0, 0, 720); // Non-Blocking version
while(robot.isMoving()) {
    printf("robot is moving ...\n");
}
printf("move finished!\n");
