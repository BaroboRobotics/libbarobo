/* Discription:
 * In this scene, six single mobots "discuss" to solve the problem.
*/
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
CMobot mobot3;
CMobot mobot4;
CMobot mobot5;
CMobot mobot6;
CMobotGroup group1;
CMobotGroup group2;

/* Connect to the mobots listed in the configuration file. */
mobot1.connect();
mobot2.connect();
mobot3.connect();
mobot4.connect();
mobot5.connect();
mobot6.connect();

/* Add the two modules to be members of our group */
group.addRobot(mobot1);
group.addRobot(mobot2);
group.addRobot(mobot3);
group.addRobot(mobot4);
group.addRobot(mobot5);
group.addRobot(mobot6);

group1.addRobot(mobot1);
group1.addRobot(mobot3);
group1.addRobot(mobot5);

group2.addRobot(mobot2);
group2.addRobot(mobot4);
group2.addRobot(mobot6);

/* Now, any commands given to "group" will cause both mobot1 and mobot2 to
 * execute the command. */
group.motionStand(); /* Cause both mobots to stand */

// discuss
group1.moveJointToNB(MOBOT_JOINT1, 90+45);
group2.moveJointToNB(MOBOT_JOINT1, -90+45);
group1.moveWait();
group2.moveWait();

group1.moveJointToNB(MOBOT_JOINT3, 90);
group2.moveJointToNB(MOBOT_JOINT3, 90);
group1.moveWait();
group2.moveWait();

group1.moveJointToNB(MOBOT_JOINT3, 70);
group2.moveJointToNB(MOBOT_JOINT3, 70);
group1.moveWait();
group2.moveWait();

group1.moveJointToNB(MOBOT_JOINT1, 45);
group2.moveJointToNB(MOBOT_JOINT1, 45);
group1.moveWait();
group2.moveWait();
