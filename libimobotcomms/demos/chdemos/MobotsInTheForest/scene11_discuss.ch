/* Discription:
 * In this scene, six single robots "discuss" to solve the problem.
*/
#include <mobot.h>
CMobot robot1;
CMobot robot2;
CMobot robot3;
CMobot robot4;
CMobot robot5;
CMobot robot6;
CMobotGroup group1;
CMobotGroup group2;

/* Connect to the robots listed in the configuration file. */
robot1.connect();
robot2.connect();
robot3.connect();
robot4.connect();
robot5.connect();
robot6.connect();

/* Add the two modules to be members of our group */
group.addRobot(robot1);
group.addRobot(robot2);
group.addRobot(robot3);
group.addRobot(robot4);
group.addRobot(robot5);
group.addRobot(robot6);

group1.addRobot(robot1);
group1.addRobot(robot3);
group1.addRobot(robot5);

group2.addRobot(robot2);
group2.addRobot(robot4);
group2.addRobot(robot6);

/* Now, any commands given to "group" will cause both robot1 and robot2 to
 * execute the command. */
group.motionStand(); /* Cause both robots to stand */

// discuss
group1.moveJointToNB(ROBOT_JOINT1, 90+45);
group2.moveJointToNB(ROBOT_JOINT1, -90+45);
group1.moveWait();
group2.moveWait();

group1.moveJointToNB(ROBOT_JOINT3, 90);
group2.moveJointToNB(ROBOT_JOINT3, 90);
group1.moveWait();
group2.moveWait();

group1.moveJointToNB(ROBOT_JOINT3, 70);
group2.moveJointToNB(ROBOT_JOINT3, 70);
group1.moveWait();
group2.moveWait();

group1.moveJointToNB(ROBOT_JOINT1, 45);
group2.moveJointToNB(ROBOT_JOINT1, 45);
group1.moveWait();
group2.moveWait();
