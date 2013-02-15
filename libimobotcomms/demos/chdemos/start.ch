/* Filename: start.ch 
 * Move the mobot faceplates. */
#include <mobot.h>

CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
printf("Reset to zero...\n");
mobot.resetToZero();

/* Rotate each of the faceplates by 360 degrees */
printf("move...\n");
mobot.move(360, 0, 360, 360);
printf("done.\n");
