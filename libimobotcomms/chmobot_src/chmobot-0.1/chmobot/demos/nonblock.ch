/* File: nonblock.ch
   use the non-blocking functoin move() . */
#include <mobot.h>

CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

robot.moveToZero();
/* Rotate each of the faceplates by 720 degrees */

//robot.move(deg2rad(720), 0, 0, deg2rad(720)); // Blocking version
robot.moveNB(deg2rad(720), 0, 0, 2*deg2rad(720)); // Non-Blocking version
while(robot.isMoving()) {
    printf("robot is moving ...\n");
}
printf("move finished!\n");
