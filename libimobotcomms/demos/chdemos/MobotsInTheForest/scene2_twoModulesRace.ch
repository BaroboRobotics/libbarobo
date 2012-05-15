/* Discription:
 * In this scene, two single robots race each other.
 */
 
#include <mobot.h>
CMobot robot1;
CMobot robot2;

/* Connect to the paired Mobot */
robot1.connect();
robot2.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot1.moveToZero();
robot2.moveToZero();

robot1.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
robot2.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);

robot1.motionInchwormRightNB(10);
robot2.motionInchwormRightNB(10);
robot1.motionWait();
robot2.motionWait();
