/* Discription:
 * In this scene, five single mobots dance.
*/
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
CMobot mobot3;
CMobot mobot4;
CMobot mobot5;
CMobotGroup group;

/* Connect to the mobots listed in the configuration file. */
mobot1.connect();
mobot2.connect();
mobot3.connect();
mobot4.connect();
mobot5.connect();

/* Add the two modules to be members of our group */
group.addRobot(mobot1);
group.addRobot(mobot2);
group.addRobot(mobot3);
group.addRobot(mobot4);
group.addRobot(mobot5);

/* Now, any commands given to "group" will cause both mobot1 and mobot2 to
 * execute the command. */
group.motionInchwormLeft(2); /* Cause both mobots to inchworm left 2 times */
group.motionInchwormRight(2);
group.motionStand(); /* Cause both mobots to stand */
group.move(360, 0, 0, 360);
delay(3); /* Make the mobots stand still for 3 seconds */
group.motionUnstand(); /* Make the mobots get back down from standing */
