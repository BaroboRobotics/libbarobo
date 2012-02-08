/* Filename: motion.ch 
   pre-programmed motions */

#include <mobot.h>

CMobot robot;

robot.connect();
robot.moveToZero();

robot.setJointSpeedRatio(ROBOT_JOINT1, 0.3);
robot.setJointSpeedRatio(ROBOT_JOINT2, 0.3);
robot.setJointSpeedRatio(ROBOT_JOINT3, 0.3);
robot.setJointSpeedRatio(ROBOT_JOINT4, 0.3);

/* pre-programmed motions */
robot.motionArchNB(15);
sleep(1);
robot.motionRollForwardNB(360);
robot.motionWait();

robot.motionInchwormLeft(2);
robot.motionInchwormRight(2);
robot.motionRollForward(360);
robot.motionRollBackward(360);
robot.motionTurnRight(360);
robot.motionTurnLeft(360);
robot.motionStand();

/* Spin the robot around two revolutions while spinning the top */

robot.move(2*360, 0, 0, 2*360);
robot.motionUnstand();
robot.motionTumble(2);

