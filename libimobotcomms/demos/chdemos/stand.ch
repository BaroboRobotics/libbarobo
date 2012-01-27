/* Filename: stand.ch 
 * Make a MoBot stand up on a faceplate */
#include <mobot.h>

CMobot robot;
/* Connect to the paired MoBot */
robot.connect();
/* Run the built-in motionStand function */
robot.motionStand();
sleep(3); // Stand still for three seconds
/* Spin the robot around one revolution while spinning the top faceplate*/
robot.move(deg2rad(720), 0, 0, deg2rad(720));
/* Lay the robot back down */
robot.motionUnstand();
