/* Filename: robotGroup.ch
 * Control multiple MoBot modules simultaneously using the CMobotGroup class */

#include <mobot.h>

CMobot robot1;
CMobot robot2;

CMobotGroup robotGroup;

/* Connect to the robots listed in the configuration file. */
robot1.connect();
robot2.connect();

/* Add the two modules to be members of our group */
robotGroup.addRobot(robot1);
robotGroup.addRobot(robot2);

/* Now, any commands given to "robotGroup" will cause both robot1 and robot2 to
 * execute the command. */
robotGroup.motionInchwormLeft(); /* Causes both robots to inchworm left */
robotGroup.motionStand(); /* Causes both robots to stand */
