/* Filename: bow.ch
 * make four mobots to bow at the same time.
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

mobot1.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
mobot2.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
mobot3.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
mobot4.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);

/* Add the four modules as the members of the group */
group.addRobot(mobot1);
group.addRobot(mobot2);
group.addRobot(mobot3);
group.addRobot(mobot4);

// bow
group.moveTo(45, 0, 0, 0);
group.moveTo(45, -60, 90, 0);
group.moveTo(45, 0, 0, 0);
