/* Filename: tankWithGripper.ch
   Control three modules as a tank with a gripper.
the vertical view of the tank
          1st 
 |---------|--------|
1|    2    |   3   ^| 4
 |---------|--------|
       |-------|
       |   1   | 3rd
       |-------|
 |---------|--------|
1|    2    |   3   ^| 4
 |---------|--------|            
          2nd                        
1. The symbol ^ indicates the switch on robots.

2. The joint 1 of the third robot is the connecting point and the switch of the third robot
is near the second robot.

3. Please make sure that the gripper is open when it is assembled.
*/
#include <mobot.h>
CMobot robot1;
CMobot robot2;
CMobot robot3;
int i;
/* Connect robot variables to the robot modules. */
robot1.connect();
robot2.connect();
robot3.connect();

robot1.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
robot2.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
robot3.setJointSpeedRatios(0.6, 0.6, 0.6, 0.8);

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot3.moveToZeroNB();
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();

/* move backward to get the "gun" */
robot1.motionRollBackwardNB(180);
robot2.motionRollBackwardNB(180);
robot1.motionWait();
robot2.motionWait();

/* "gun" ready */
robot3.moveTo(0, 90, 0, 0);
robot3.moveJoint(MOBOT_JOINT4, 4*360);
robot3.moveTo(0, 90, 90, 0);
//robot3.moveTo(0, 90, 45, 0);
robot3.moveJoint(MOBOT_JOINT1, 360);

/* move forward */
robot1.motionRollForwardNB(180);
robot2.motionRollForwardNB(180);
robot1.motionWait();
robot2.motionWait();
/* move backward */
robot1.motionRollBackwardNB(180);
robot2.motionRollBackwardNB(180);
robot1.motionWait();
robot2.motionWait();

/* turn right */
robot1.motionTurnRightNB(360);
robot2.motionTurnRightNB(360);
robot1.motionWait();
robot2.motionWait();

/* turn left */
robot1.motionTurnLeftNB(360);
robot2.motionTurnLeftNB(360);
robot1.motionWait();
robot2.motionWait();
