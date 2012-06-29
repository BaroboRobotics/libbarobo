/* Discription:
 * In this scene, one mobot do different motions.
 */
#include <mobot.h>
CMobot mobot1;
mobot1.connect();
mobot1.moveToZeroNB();

// set mobots' speed
mobot1.setJointSpeedRatios(0.8, 0.8, 0.8, 0.8);

mobot1.motionTurnRight();
mobot1.motionRollForward(3*360);
mobot1.motionTurnLeft();
mobot5.motionTumbleLeft(5);
