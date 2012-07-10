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
1. The symbol ^ indicates the switch on mobots.

2. The joint 1 of the third mobot is the connecting point and the switch of the third mobot
is near the second mobot.

3. Please make sure that the gripper is open when it is assembled.
*/
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
CMobot mobot3;
int i;
/* Connect mobot variables to the mobot modules. */
mobot1.connect();
mobot2.connect();
mobot3.connect();

mobot1.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
mobot2.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
mobot3.setJointSpeedRatios(0.6, 0.6, 0.6, 0.8);

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot1.resetToZeroNB();
mobot2.resetToZeroNB();
mobot3.resetToZeroNB();
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();

/* move backward to get the "gun" */
mobot1.motionRollBackwardNB(180);
mobot2.motionRollBackwardNB(180);
mobot1.motionWait();
mobot2.motionWait();

/* "gun" ready */
mobot3.moveTo(0, 90, 0, 0);
mobot3.moveJoint(MOBOT_JOINT4, 4*360);
mobot3.moveTo(0, 90, 90, 0);
//mobot3.moveTo(0, 90, 45, 0);
mobot3.moveJoint(MOBOT_JOINT1, 360);

/* move forward */
mobot1.motionRollForwardNB(180);
mobot2.motionRollForwardNB(180);
mobot1.motionWait();
mobot2.motionWait();
/* move backward */
mobot1.motionRollBackwardNB(180);
mobot2.motionRollBackwardNB(180);
mobot1.motionWait();
mobot2.motionWait();

/* turn right */
mobot1.motionTurnRightNB(360);
mobot2.motionTurnRightNB(360);
mobot1.motionWait();
mobot2.motionWait();

/* turn left */
mobot1.motionTurnLeftNB(360);
mobot2.motionTurnLeftNB(360);
mobot1.motionWait();
mobot2.motionWait();
