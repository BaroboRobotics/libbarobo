/* Discription:
 * In this scene, there are five one-module mobots do different motions.
 */
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
CMobot mobot3;
CMobot mobot4;
CMobot mobot5;

mobot1.connect();
mobot2.connect();
mobot3.connect();
mobot4.connect();
mobot5.connect();

// initialization
mobot1.moveToZeroNB();
mobot2.moveToZeroNB();
mobot3.moveToNB(0, 90, 90, 0);
mobot4.moveToZeroNB();
mobot5.moveToZeroNB();
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
mobot4.moveWait();
mobot5.moveWait();

// set mobots' speed
mobot1.setJointSpeedRatios(0.8, 0.8, 0.8, 0.8);
mobot2.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
mobot3.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
mobot4.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
mobot5.setJointSpeedRatios(0.8, 0.8, 0.8, 0.8);

// mobot four do stand up
mobot4.motionStand();
// mobot one do inch worm right
mobot1.motionInchwormRightNB(10);
// roobt two do roll forward
mobot2.motionRollForwardNB(3*360);
// mobot three do roll forward with skinny type
mobot3.motionRollForwardNB(3*360);
mobot4.moveNB(3*360, 0, 0, 3*360);
// mobot five do tumble left
mobot5.motionTumbleLeft(5);
mobot1.motionWait();
mobot2.motionWait();
mobot3.motionWait();
mobot4.moveWait();
mobot5.motionWait();
