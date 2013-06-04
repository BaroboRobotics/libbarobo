/* Discription:
 * In this scene, six modules are assembled as two three-module tanks with grippers.
 *          1st 
 * |---------|--------|
 *1|    2    |   3   ^| 4
 * |---------|--------|
 *       |-------|
 *       |   1   | 3rd
 *       |-------|
 * |---------|--------|
 *1|    2    |   3   ^| 4
 * |---------|--------|            
 *          2nd                        
 *1. The symbol ^ indicates the switche on mobots.
 *
 *2. The joint 1 of the third mobot is the connecting point and the switch of the third mobot
 *is near the second mobot.
 *
 *3. Please make sure that the gripper is open when it is assembled.
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

/* Connect mobot variables to the mobot modules. */
mobot1.connect();
mobot2.connect();
mobot3.connect();
mobot4.connect();
mobot5.connect();
mobot6.connect();

mobot1.setJointSpeedRatios(0.8, 0.4, 0.4, 0.8);
mobot2.setJointSpeedRatios(0.8, 0.4, 0.4, 0.8);
mobot3.setJointSpeedRatios(0.6, 0.6, 0.6, 0.8);
mobot4.setJointSpeedRatios(0.8, 0.4, 0.4, 0.8);
mobot5.setJointSpeedRatios(0.8, 0.4, 0.4, 0.8);
mobot6.setJointSpeedRatios(0.6, 0.6, 0.6, 0.8);

/* Add the two modules to be members of our group */
group1.addRobot(mobot1);
group1.addRobot(mobot2);
group1.addRobot(mobot4);
group1.addRobot(mobot5);
group2.addRobot(mobot3);
group2.addRobot(mobot6);

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
group1.resetToZeroNB();
group2.resetToZeroNB();
group1.moveWait();
group2.moveWait();

// "gun" ready
group2.moveTo(0, 90, 0, 0);
group2.moveJointTo(ROBOT_JOINT4, 4*360);
group2.moveTo(0, 90, 60, 0);

// move forward
group1.motionRollForward(360);

// turn right
group1.motionTurnRight(270);

// move backward
group1.motionRollBackward(180);

// turn left
group1.motionTurnLeft(360);
