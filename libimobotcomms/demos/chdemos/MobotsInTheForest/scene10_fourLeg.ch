/* Discription:
 * In this scene, four modules are assembled as an fourLeg robot.
 *          1st                        3rd
 * |---------|--------|   |   |---------|--------|
 *1|    2    |    3   |4 X|X 1|    2    |   3    |4
 * |---------|--------|   |   |---------|--------|
 *          2nd           |            4th
 * |---------|--------|   |   |---------|--------|
 *1|    2    |    3   |4 X|X 1|    2    |   3    |4
 * |---------|--------|   |   |---------|--------|
 *Before assembling, please make sure that each robot is in zero position. 
 *Note: Push the buttom B to make them go to zero position.
 */
 
#include <mobot.h>
CMobot robot1;
CMobot robot2;
CMobot robot3;
CMobot robot4;
CMobotGroup group1;
CMobotGroup group2;

/* Connect to the robots listed in the configuration file. */
robot1.connect();
robot2.connect();
robot3.connect();
robot4.connect();

/* Add the two modules to be members of our group */
group1.addRobot(robot1);
group1.addRobot(robot2);
group2.addRobot(robot3);
group2.addRobot(robot4);

// move to zero position
group1.moveToZeroNB();
group2.moveToZeroNB();
group1.moveWait();
group2.moveWait();

// set robots speed
robot1.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
robot2.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
robot3.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
robot4.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);

// get ready to roll
group1.moveToNB(0, 90, 90, 0);
group2.moveToNB(0, -90, -90, 0);
group1.moveWait();
group2.moveWait();

// move forward
group1.moveJointToNB(ROBOT_JOINT1, 2*360);
group2.moveJointToNB(ROBOT_JOINT4, 2*360);
group1.moveWait();
group2.moveWait();
