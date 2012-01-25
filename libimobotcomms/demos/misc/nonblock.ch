/* File: nonblock.ch
   use the non-blocking functoin move() . */
#include <mobot.h>

CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

robot.moveToZero();
/* Rotate each of the faceplates by 90 degrees */

//robot.move(5*deg2rad(360), 0, 0, 5*deg2rad(360)); // Blocking version
robot.moveNB(2*deg2rad(360), 0, 0, 2*deg2rad(360)); // Non-Blocking version
while(robot.isMoving() == true) {
    printf("robot is moving ...\n");
}
printf("move finished!\n");
