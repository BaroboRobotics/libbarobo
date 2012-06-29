/* Filename: fourLegs.ch
 * Control multiple Mobot modules simultaneously using the CMobotGroup class. 
   You may place the four mobots as follows:
           1st                        3rd
  |---------|--------|   |   |---------|--------|
 1|    2    |    3   |4 X|X 1|    2    |   3    |4
  |---------|--------|   |   |---------|--------|
           2nd           |            4th
  |---------|--------|   |   |---------|--------|
 1|    2    |    3   |4 X|X 1|    2    |   3    |4
  |---------|--------|   |   |---------|--------|
Before assembling, please make sure that each mobot is in zero position. 
Note: Push the buttom B to make them go back to zero position.
*/
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
CMobot mobot3;
CMobot mobot4;
CMobotGroup group1;
CMobotGroup group2;

/* Connect to the mobots listed in the configuration file. */
mobot1.connect();
mobot2.connect();
mobot3.connect();
mobot4.connect();

/* Add the two modules to be members of our group */
group1.addRobot(mobot1);
group1.addRobot(mobot2);
group2.addRobot(mobot3);
group2.addRobot(mobot4);

// move to zero position
group1.moveToZeroNB();
group2.moveToZeroNB();
group1.moveWait();
group2.moveWait();

// set mobots speed
mobot1.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
mobot2.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
mobot3.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
mobot4.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);

// lift
group1.moveToNB(0, -90, 0, 0);
group2.moveToNB(0, 0, 90, 0);
group1.moveWait();
group2.moveWait();

mobot1.moveToNB(0, -90, 0, 38);
mobot2.moveToNB(0, 0, 90, 38);
mobot3.moveToNB(38, 0, 90, 0);
mobot4.moveToNB(38, -90, 0, 0);
mobot1.moveWait();
mobot3.moveWait();
mobot2.moveWait();
mobot4.moveWait();

group1.moveToNB(0, 0, 90, 0);
group2.moveToNB(0, -90, 0, 0);
group1.moveWait();
group2.moveWait();

// kick
mobot1.moveTo(0, 0, 0, 0);
mobot1.moveTo(0, 0, 90, 0);

mobot2.moveTo(0, 0, 0, 0);
mobot2.moveTo(0, 0, 90, 0);

mobot3.moveTo(0, 0, 0, 0);
mobot3.moveTo(0,-90, 0, 0);

mobot4.moveTo(0, 0, 0, 0);
mobot4.moveTo(0, -90, 0, 0);

// two legs kick
mobot1.moveToNB(0, 0, 0, 0);
mobot4.moveToNB(0, 0, 0, 0);
mobot1.moveWait();
mobot4.moveWait();

mobot1.moveJointNB(MOBOT_JOINT4, -360);
mobot4.moveJointNB(MOBOT_JOINT1, 360);
mobot1.moveWait();
mobot2.moveWait();

mobot1.moveToNB(0, 0, 90, 0);
mobot4.moveToNB(0, -90, 0, 0);
mobot1.moveWait();
mobot4.moveWait();

mobot2.moveToNB(0, 0, 0, 0);
mobot3.moveToNB(0, 0, 0, 0);
mobot2.moveWait();
mobot3.moveWait();

mobot2.moveJointNB(MOBOT_JOINT4, 360);
mobot3.moveJointNB(MOBOT_JOINT1, -360);
mobot2.moveWait();
mobot3.moveWait();

mobot2.moveToNB(0, 0, 90, 0);
mobot3.moveToNB(0, -90, 0, 0);
mobot2.moveWait();
mobot3.moveWait();

// get ready to roll
group1.moveToNB(0, 90, 90, 0);
group2.moveToNB(0, -90, -90, 0);
group1.moveWait();
group2.moveWait();

// move forward
group1.moveJointToNB(MOBOT_JOINT1, 360);
group2.moveJointToNB(MOBOT_JOINT4, 360);
group1.moveWait();
group2.moveWait();

// move backward
group1.moveJointToNB(MOBOT_JOINT1, -360);
group2.moveJointToNB(MOBOT_JOINT4, -360);
group1.moveWait();
group2.moveWait();

// turn left
group1.moveJointToNB(MOBOT_JOINT1, -360);
group2.moveJointToNB(MOBOT_JOINT4, 360);
group1.moveWait();
group2.moveWait();

// turn right
group1.moveJointToNB(MOBOT_JOINT1, 360);
group2.moveJointToNB(MOBOT_JOINT4, -360);
group1.moveWait();
group2.moveWait();

// move back to zero position
group1.moveToNB(0, 0, 0, 0);
group2.moveToNB(0, 0, 0, 0);
group1.moveWait();
group2.moveWait();
