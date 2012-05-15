/* Discription:
 * In this scene, there are five one-module robots do different motions.
 */
#include <mobot.h>
CMobot robot1;
CMobot robot2;
CMobot robot3;
CMobot robot4;
CMobot robot5;

robot1.connect();
robot2.connect();
robot3.connect();
robot4.connect();
robot5.connect();

// initialization
robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot3.moveToNB(0, 90, 90, 0);
robot4.moveToZeroNB();
robot5.moveToZeroNB();
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
robot4.moveWait();
robot5.moveWait();

// set robots' speed
robot1.setJointSpeedRatios(0.8, 0.8, 0.8, 0.8);
robot2.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot3.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot4.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot5.setJointSpeedRatios(0.8, 0.8, 0.8, 0.8);

// robot four do stand up
robot4.motionStand();
// robot one do inch worm right
robot1.motionInchwormRightNB(10);
// roobt two do roll forward
robot2.motionRollForwardNB(3*360);
// robot three do roll forward with skinny type
robot3.motionRollForwardNB(3*360);
robot4.moveNB(3*360, 0, 0, 3*360);
// robot five do tumble left
robot5.motionTumbleLeft(5);
robot1.motionWait();
robot2.motionWait();
robot3.motionWait();
robot4.moveWait();
robot5.motionWait();
