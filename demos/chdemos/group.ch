/* Filename: group.ch
 * Control multiple Mobot modules simultaneously using the CMobotGroup class */
#include <mobot.h>
#define NUM_ROBOTS 13
CLinkbotI robots[NUM_ROBOTS];
CLinkbotIGroup group;

/* Add the two modules to be members of our group */
group.addRobots(robots);
group.connect();

/* Now, any commands given to "group" will cause both robot1 and robot2 to
 * execute the command. */
group.move(360, 0, 360);   /* Joints 1 and 4 rotate 360 degrees */
delay(3);                     /* Mobots stand still for 3 seconds */
group.move(360, 0, 360);   /* Joints 1 and 4 rotate 360 degrees */
