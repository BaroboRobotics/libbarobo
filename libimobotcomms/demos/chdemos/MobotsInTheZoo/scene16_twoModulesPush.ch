/* Discription:
 * In this scene, two oneModule mobots push twoWheelDrive.
 */
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
CMobot mobot3;

mobot1.connect();
mobot2.connect();
mobot3.connect();

// initialization
mobot1.moveToZeroNB();
mobot2.moveToZeroNB();
mobot3.moveToZeroNB();

// set mobots' speed
mobot1.setJointSpeedRatios(0.8, 0.8, 0.8, 0.8);
mobot2.setJointSpeedRatios(0.8, 0.8, 0.8, 0.8);
mobot3.setJointSpeedRatios(0.05, 0.05, 0.05, 0.05);

// mobot one and two inch worm right to push, mobot three move back
mobot1.motionInchwormRightNB(20);
mobot2.motionInchwormRightNB(20);
mobot3.motionRollBackward(360);
mobot1.motionWait();
mobot2.motionWait();
mobot3.motionWait();
