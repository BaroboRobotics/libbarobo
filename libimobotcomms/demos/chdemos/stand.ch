/* Filename: stand.ch 
 * Make a Mobot stand up on a faceplate */
#include <mobot.h>
CMobot mobot;

/* Connect to the Mobot */
mobot.connect();
/* Run the built-in motionStand function */
mobot.motionStand();
delay(3); // Stand still for three seconds
/* Spin the mobot around two revolutions while spinning the top faceplate*/
mobot.move(2*360, 0, 0, 2*360);
/* Lay the mobot back down */
mobot.motionUnstand();
