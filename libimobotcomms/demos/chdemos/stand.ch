/* Filename: stand.ch 
 * Make a Mobot stand up on a faceplate */
#include <mobot.h>
CMobot robot;

/* Connect to the Mobot */
robot.connect();
/* Run the built-in motionStand function */
robot.motionStand();
delay(3); // Stand still for three seconds
/* Spin the robot around two revolutions while spinning the top faceplate*/
robot.move(2*360, 0, 0, 2*360);
/* Lay the robot back down */
robot.motionUnstand();
