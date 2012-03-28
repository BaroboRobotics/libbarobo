/* Filename: motion.ch 
   pre-programmed motions */
#include <mobot.h>
CMobot robot;

robot.connect();
robot.moveToZero();

/* set to 50% of the maximum speed */
robot.setJointSpeedRatios(0.5, 0.5, 0.5, 0.5);

/* pre-programmed motions */
robot.motionInchwormLeft(2);
robot.motionInchwormRight(2);
robot.motionRollForward(360);
robot.motionRollBackward(360);
robot.motionTurnRight(360);
robot.motionTurnLeft(360);

robot.motionStand(); // stand up
/* Spin the robot around two revolutions while spinning the top */
robot.move(2*360, 0, 0, 2*360);
robot.motionUnstand();
robot.motionTumbleLeft(2);
