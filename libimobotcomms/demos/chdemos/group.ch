/* Filename: group.ch
 * Control multiple Mobot modules simultaneously using the CMobotGroup class */
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
group.motionInchwormLeft(4);  /* Both robots inchworm left 4 times */
group.motionStand();          /* Both robots stand */
group.move(360, 0, 0, 360);   /* Joints 1 and 4 rotate 360 degrees */
sleep(3);                     /* Robots stand still for 3 seconds */
group.motionUnstand();        /* Robots get back down from standing */
