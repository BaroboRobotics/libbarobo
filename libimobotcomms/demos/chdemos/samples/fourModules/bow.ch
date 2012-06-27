/* Filename: bow.ch
 * make four mobots to bow at the same time.
*/
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

robot1.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot2.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot3.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot4.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);

/* Add the four modules as the members of the group */
group.addMobot(robot1);
group.addMobot(robot2);
group.addMobot(robot3);
group.addMobot(robot4);

// bow
group.moveTo(45, 0, 0, 0);
group.moveTo(45, -60, 90, 0);
group.moveTo(45, 0, 0, 0);
