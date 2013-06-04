/* Discription:
 * In this scene, four modules are assembled as an fourLeg mobot.
 *          1st                        3rd
 * |---------|--------|   |   |---------|--------|
 *1|    2    |    3   |4 X|X 1|    2    |   3    |4
 * |---------|--------|   |   |---------|--------|
 *          2nd           |            4th
 * |---------|--------|   |   |---------|--------|
 *1|    2    |    3   |4 X|X 1|    2    |   3    |4
 * |---------|--------|   |   |---------|--------|
 *Before assembling, please make sure that each mobot is in zero position. 
 *Note: Push the buttom B to make them go to zero position.
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
group1.resetToZeroNB();
group2.resetToZeroNB();
group1.moveWait();
group2.moveWait();

// set mobots speed
mobot1.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
mobot2.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
mobot3.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
mobot4.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);

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
