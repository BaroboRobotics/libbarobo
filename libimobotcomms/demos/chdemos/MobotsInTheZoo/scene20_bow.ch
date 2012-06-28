/* Discription:
 * In this scene, seven single mobots bow.
*/
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
CMobot mobot3;
CMobot mobot4;
CMobot mobot5;
CMobot mobot6;
CMobot mobot7;
CMobotGroup group;

/* Connect to the mobots listed in the configuration file. */
mobot1.connect();
mobot2.connect();
mobot3.connect();
mobot4.connect();
mobot5.connect();
mobot6.connect();
mobot7.connect();

mobot1.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
mobot2.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
mobot3.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
mobot4.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
mobot5.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
mobot6.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
mobot7.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);

/* Add the two modules to be members of our group */
group.addRobot(mobot1);
group.addRobot(mobot2);
group.addRobot(mobot3);
group.addRobot(mobot4);
group.addRobot(mobot5);
group.addRobot(mobot6);
group.addRobot(mobot7);

// bow
group.moveTo(45, 0, 0, 0);
group.moveTo(45, -60, 90, 0);
group.moveTo(45, 0, 0, 0);
