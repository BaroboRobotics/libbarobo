/* Filename: motion.ch 
 * Move the two wheeled robot. */
#include <mobot.h>
CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

/* test all pre-programmed motions */
robot.motionArch(90);
robot.motionInchwormLeft(4);
robot.motionInchwormRight(4);
robot.motionRollBackward(360);
robot.motionRollForward(360);
robot.motionTurnLeft(360);
robot.motionTurnRight(360);
robot.motionStand();
robot.move(360, 0, 0, 360);
robot.motionUnstand();
robot.motionTumble(2);
