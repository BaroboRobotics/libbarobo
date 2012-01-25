/* Filename: motion.ch 
 * Move the two wheeled robot. */

#include <mobot.h>

CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

/* test all pre-programmed motions */
robot.motionInchwormLeft(1);
robot.motionInchwormRight(1);
robot.motionRollBackward(deg2rad(360));
robot.motionRollForward(deg2rad(360));
robot.motionTurnLeft(deg2rad(360));
robot.motionTurnRight(deg2rad(360));
robot.motionStand();
robot.motionUnstand();
