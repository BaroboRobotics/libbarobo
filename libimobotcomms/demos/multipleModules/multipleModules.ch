/* Filename: multipleModules.ch
 * Control two modules and make them stand simultaneously. */

#include <mobot.h>

CMobot robot1;
CMobot robot2;

/* For multiple robots, we need to use the "connectWithAddress" function to
 * connect to separate robots. Substitute the string "11:11:11:11:11:11" with
 * the address of the first MoBot and the string "22:22:22:22:22:22" with the
 * string of the second MoBot.*/
int defaultChannel = 1;
string_t bluetoothAddress1 = "11:11:11:11:11:11";
string_t bluetoothAddress2 = "22:22:22:22:22:22";
robot1.connectWithAddress(bluetoothAddress1, defaultChannel);
robot2.connectWithAddress(bluetoothAddress2, defaultChannel);

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot1.moveToZeroNB();
robot2.moveToZeroNB();

robot1.moveWait();
robot2.moveWait();

/* Make both robots stand simultaneously. Note that we must use the
 * non-blocking versions of the motion functions here in order for the robots
 * to perform the motions simultaneously. */
robot1.motionStandNB();
robot2.motionStandNB();
robot1.moveWait();
robot2.moveWait();

