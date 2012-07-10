/* File: nonblock.ch
   use the non-blocking functoin moveNB() . */
#include <mobot.h>
CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();

mobot.resetToZero();

/* Rotate each of the faceplates by 720 degrees */
//mobot.move(720, 0, 0, 720); // Blocking version
mobot.moveNB(720, 0, 0, 720); // Non-Blocking version
while(mobot.isMoving()) {
    printf("mobot is moving ...\n");
}
printf("move finished!\n");
