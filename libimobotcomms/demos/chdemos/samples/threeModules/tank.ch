/* Filename: tank.ch
   Control three modules as a tank.
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
The symbol ^ indicates the switch on robots.
The joint 1 of the third robot is the connecting point and the switch of the third robot
is near the second robot.
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
robot3.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot3.moveToZeroNB();
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();

/* "gun" ready */
robot3.moveJoint(MOBOT_JOINT2, -90);

/* "gun" scan */
robot3.moveTo(360, -90, 0, 0);

/* move forward */
robot1.motionRollForwardNB(360);
robot2.motionRollForwardNB(360);
robot3.moveToNB(360, -90, 0, 0);
robot1.motionWait();
robot2.motionWait();
robot3.moveWait();

/* move backward */
robot1.motionRollForwardNB(-360);
robot2.motionRollForwardNB(-360);
robot3.moveToNB(-360, -90, 0, 0);
robot1.motionWait();
robot2.motionWait();
robot3.moveWait();

/* moving the "gun"*/
robot3.moveTo(0, -45, 45, 0);
robot3.moveTo(360, -45, 45, 0);
robot3.moveTo(0, 0, 90, 0);
robot3.moveTo(360, 0, 90, 0);
robot3.moveTo(0, 0, 0, 0);
robot3.moveTo(360, 0, 0, 0);

/* turn right */
robot1.motionTurnRightNB(180);
robot2.motionTurnRightNB(180);
robot3.moveToNB(-90, -90, 0, 0);
robot1.motionWait();
robot2.motionWait();
robot3.moveWait();

/* turn left */
robot1.motionTurnLeftNB(180);
robot2.motionTurnLeftNB(180);
robot3.moveToNB(90, -90, 0, 0);
robot1.motionWait();
robot2.motionWait();
robot3.moveWait();
