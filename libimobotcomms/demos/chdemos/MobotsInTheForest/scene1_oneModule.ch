/* Discription:
 * In this scene, one robot do different motions.
 */
#include <mobot.h>
CMobot robot1;
robot1.connect();
robot1.moveToZeroNB();

// set robots' speed
robot1.setJointSpeedRatios(0.8, 0.8, 0.8, 0.8);

robot1.motionTurnRight();
robot1.motionRollForward(3*360);
robot1.motionTurnLeft();
robot5.motionTumbleLeft(5);
