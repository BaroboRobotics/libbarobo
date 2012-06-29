/* Filename: group.ch
 * Control multiple Mobot modules simultaneously using the CMobotGroup class */
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
CMobotGroup group;

/* Connect to the mobots listed in the configuration file. */
mobot1.connect();
mobot2.connect();

/* Add the two modules to be members of our group */
group.addRobot(mobot1);
group.addRobot(mobot2);

/* Now, any commands given to "group" will cause both mobot1 and mobot2 to
 * execute the command. */
group.motionInchwormLeft(4);  /* Both mobots inchworm left 4 times */
group.motionStand();          /* Both mobots stand */
group.move(360, 0, 0, 360);   /* Joints 1 and 4 rotate 360 degrees */
delay(3);                     /* Mobots stand still for 3 seconds */
group.motionUnstand();        /* Mobots get back down from standing */
