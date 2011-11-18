/* Filename: stand2.ch 
 * Make a MoBot stand up on a faceplate */
#include <mobot.h>

CMobot robot;
/* Connect to the paired MoBot */
robot.connect();
/* Run the built-in motionStand function */
robot.motionStand();

