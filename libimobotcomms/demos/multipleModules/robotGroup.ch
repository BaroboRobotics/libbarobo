/* Filename: robotGroup.ch
 * Control multiple MoBot modules simultaneously using the CMobotGroup class */

#include <mobot.h>

CMobot robot1;
CMobot robot2;

CMobotGroup robotGroup;

/* First, connect the individual robots. */
int defaultChannel = 1;
string_t bluetoothAddress1 = "11:11:11:11:11:11";
string_t bluetoothAddress2 = "22:22:22:22:22:22";
robot1.connectWithAddress(bluetoothAddress1, defaultChannel);
robot2.connectWithAddress(bluetoothAddress2, defaultChannel);

/* Add the two modules to be members of our group */
robotGroup.addRobot(robot1);
robotGroup.addRobot(robot2);

/* Now, any commands given to "robotGroup" will cause both robot1 and robot2 to
 * execute the command. */
robotGroup.motionInchwormLeft(); /* Causes both robots to inchworm left */
robotGroup.motionStand(); /* Causes both robots to stand */
