/* Discription:
 * In this scene, seven single robots bow.
*/
#include <mobot.h>
CMobot robot1;
CMobot robot2;
CMobot robot3;
CMobot robot4;
CMobot robot5;
CMobot robot6;
CMobot robot7;
CMobotGroup group;

/* Connect to the robots listed in the configuration file. */
robot1.connect();
robot2.connect();
robot3.connect();
robot4.connect();
robot5.connect();
robot6.connect();
robot7.connect();

robot1.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot2.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot3.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot4.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot5.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot6.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot7.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);

/* Add the two modules to be members of our group */
group.addRobot(robot1);
group.addRobot(robot2);
group.addRobot(robot3);
group.addRobot(robot4);
group.addRobot(robot5);
group.addRobot(robot6);
group.addRobot(robot7);

// bow
group.moveTo(45, 0, 0, 0);
group.moveTo(45, -60, 90, 0);
group.moveTo(45, 0, 0, 0);
