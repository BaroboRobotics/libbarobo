/* Filename: dance.ch
 * Control multiple MoBot modules simultaneously using the CMobotGroup class */

#include <mobot.h>

CMobot robot1;
CMobot robot2;
CMobot robot3;
CMobot robot4;

CMobotGroup group;

/* Connect to the robots listed in the configuration file. */
robot1.connect();
robot2.connect();
robot3.connect();
robot4.connect();

/* Add the two modules to be members of our group */
group.addRobot(robot1);
group.addRobot(robot2);
group.addRobot(robot3);
group.addRobot(robot4);

/* Now, any commands given to "group" will cause both robot1 and robot2 to
 * execute the command. */
group.motionInchwormLeft(2); /* Cause both robots to inchworm left 2 times */
group.motionInchwormRight(2);
group.motionStand(); /* Cause both robots to stand */
group.move(360, 0, 0, 360);
sleep(3); /* Make the robots stand still for 3 seconds */
group.motionUnstand(); /* Make the robots get back down from standing */
