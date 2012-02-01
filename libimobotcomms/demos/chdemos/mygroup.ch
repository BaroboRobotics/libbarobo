/* Filename: group.ch
 * Control multiple MoBot modules simultaneously using the CMobotGroup class */

#include <mobot.h>

CMobot robot1;
CMobot robot2;

CMobotGroup group;

/* Connect to the robots listed in the configuration file. */
robot1.connect();
robot2.connect();

/* Add the two modules to be members of our group */
group.addRobot(robot1);
group.addRobot(robot2);

/* Now, any commands given to "group" will cause both robot1 and robot2 to
 * execute the command. */
group.moveToZero();
group.motionStand(); /* Causes both robots to stand */
group.moveToZero();
group.motionInchwormLeft(1); /* Causes both robots to inchworm left */
group.motionRollForward(3);
group.motionRollBackward(3);
group.motionInchwormRightNB(1);
group.motionWait();
