/* Filename: dance.ch
 * Control multiple Mobot modules simultaneously using the CMobotGroup class. 
   You may place the four mobots as follows:
      Mobot1   Mobot2
      Mobot3   Mobot4
*/
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
CMobot mobot3;
CMobot mobot4;
CMobotGroup group;

/* Connect to the mobots listed in the configuration file. */
mobot1.connect();
mobot2.connect();
mobot3.connect();
mobot4.connect();

/* Add the two modules to be members of our group */
group.addMobot(mobot1);
group.addMobot(mobot2);
group.addMobot(mobot3);
group.addMobot(mobot4);

/* Now, any commands given to "group" will cause both mobot1 and mobot2 to
 * execute the command. */
group.motionInchwormLeft(2); /* Cause both mobots to inchworm left 2 times */
group.motionInchwormRight(2);
group.motionStand(); /* Cause both mobots to stand */
group.move(360, 0, 0, 360);
delay(3); /* Make the mobots stand still for 3 seconds */
group.motionUnstand(); /* Make the mobots get back down from standing */
