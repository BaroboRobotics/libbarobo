/* Filename: motion.ch 
 * Move the two wheeled mobot. */
#include <mobot.h>
CMobot robot;

/* Connect to the paired Mobot */
robot.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.resetToZero();

robot.moveTo(90, 0, 0, 90);

robot.moveTo(180, 0, 0, 180);

robot.moveTo(0, 0, 0, 0);

/* test all pre-programmed motions */
robot.motionArch(90);
robot.motionInchwormLeft(4);
robot.motionInchwormRight(4);
robot.moveBackward(360);
robot.moveForward(360);
robot.turnLeft(360);
robot.turnRight(360);
robot.motionStand();
robot.move(360, 0, 0, 360);
robot.motionUnstand();
robot.motionTumbleLeft(2);
