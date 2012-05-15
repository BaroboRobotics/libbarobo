/* Discription:
 * In this scene, two oneModule robots push twoWheelDrive.
 */
#include <mobot.h>
CMobot robot1;
CMobot robot2;
CMobot robot3;

robot1.connect();
robot2.connect();
robot3.connect();

// initialization
robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot3.moveToZeroNB();

// set robots' speed
robot1.setJointSpeedRatios(0.8, 0.8, 0.8, 0.8);
robot2.setJointSpeedRatios(0.8, 0.8, 0.8, 0.8);
robot3.setJointSpeedRatios(0.05, 0.05, 0.05, 0.05);

// robot one and two inch worm right to push, robot three move back
robot1.motionInchwormRightNB(20);
robot2.motionInchwormRightNB(20);
robot3.motionRollBackward(360);
robot1.motionWait();
robot2.motionWait();
robot3.motionWait();
