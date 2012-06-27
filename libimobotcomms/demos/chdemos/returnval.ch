/* Filename: returnval.ch 
 * Rotate the faceplates by 90 degrees */
#include <mobot.h>
CMobot mobot; 
double angle1, angle4;

/* Connect to the paired Mobot */
if(mobot.connect())
{
    printf("Failed to connect to the mobot.\n");
    exit(-1);
}
/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot.moveToZero();

/* Rotate each of the faceplates by 360 degrees */
angle1 = 360;
angle4 = 360;
mobot.move(angle1, 0, 0, angle4);
/* Move the motors back to where they were */
angle1 = -360;
angle4 = -360;
mobot.move(angle1, 0, 0, angle4);
