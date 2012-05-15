/* Filename: fourLegs.ch
 * Control multiple Mobot modules simultaneously using the CMobotGroup class. 
   You may place the four robots as follows:
           1st                        3rd
  |---------|--------|   |   |---------|--------|
 1|    2    |    3   |4 X|X 1|    2    |   3    |4
  |---------|--------|   |   |---------|--------|
           2nd           |            4th
  |---------|--------|   |   |---------|--------|
 1|    2    |    3   |4 X|X 1|    2    |   3    |4
  |---------|--------|   |   |---------|--------|
Before assembling, please make sure that each robot is in zero position. 
Note: Push the buttom B to make them go back to zero position.
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

// lift
group1.moveToNB(0, -90, 0, 0);
group2.moveToNB(0, 0, 90, 0);
group1.moveWait();
group2.moveWait();

robot1.moveToNB(0, -90, 0, 38);
robot2.moveToNB(0, 0, 90, 38);
robot3.moveToNB(38, 0, 90, 0);
robot4.moveToNB(38, -90, 0, 0);
robot1.moveWait();
robot3.moveWait();
robot2.moveWait();
robot4.moveWait();

group1.moveToNB(0, 0, 90, 0);
group2.moveToNB(0, -90, 0, 0);
group1.moveWait();
group2.moveWait();

// kick
robot1.moveTo(0, 0, 0, 0);
robot1.moveTo(0, 0, 90, 0);

robot2.moveTo(0, 0, 0, 0);
robot2.moveTo(0, 0, 90, 0);

robot3.moveTo(0, 0, 0, 0);
robot3.moveTo(0,-90, 0, 0);

robot4.moveTo(0, 0, 0, 0);
robot4.moveTo(0, -90, 0, 0);

// two legs kick
robot1.moveToNB(0, 0, 0, 0);
robot4.moveToNB(0, 0, 0, 0);
robot1.moveWait();
robot4.moveWait();

robot1.moveJointNB(ROBOT_JOINT4, -360);
robot4.moveJointNB(ROBOT_JOINT1, 360);
robot1.moveWait();
robot2.moveWait();

robot1.moveToNB(0, 0, 90, 0);
robot4.moveToNB(0, -90, 0, 0);
robot1.moveWait();
robot4.moveWait();

robot2.moveToNB(0, 0, 0, 0);
robot3.moveToNB(0, 0, 0, 0);
robot2.moveWait();
robot3.moveWait();

robot2.moveJointNB(ROBOT_JOINT4, 360);
robot3.moveJointNB(ROBOT_JOINT1, -360);
robot2.moveWait();
robot3.moveWait();

robot2.moveToNB(0, 0, 90, 0);
robot3.moveToNB(0, -90, 0, 0);
robot2.moveWait();
robot3.moveWait();

// get ready to roll
group1.moveToNB(0, 90, 90, 0);
group2.moveToNB(0, -90, -90, 0);
group1.moveWait();
group2.moveWait();

// move forward
group1.moveJointToNB(ROBOT_JOINT1, 360);
group2.moveJointToNB(ROBOT_JOINT4, 360);
group1.moveWait();
group2.moveWait();

// move backward
group1.moveJointToNB(ROBOT_JOINT1, -360);
group2.moveJointToNB(ROBOT_JOINT4, -360);
group1.moveWait();
group2.moveWait();

// turn left
group1.moveJointToNB(ROBOT_JOINT1, -360);
group2.moveJointToNB(ROBOT_JOINT4, 360);
group1.moveWait();
group2.moveWait();

// turn right
group1.moveJointToNB(ROBOT_JOINT1, 360);
group2.moveJointToNB(ROBOT_JOINT4, -360);
group1.moveWait();
group2.moveWait();

// move back to zero position
group1.moveToNB(0, 0, 0, 0);
group2.moveToNB(0, 0, 0, 0);
group1.moveWait();
group2.moveWait();
