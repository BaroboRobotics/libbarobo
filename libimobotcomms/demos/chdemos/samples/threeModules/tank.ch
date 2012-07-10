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
The symbol ^ indicates the switch on mobots.
The joint 1 of the third mobot is the connecting point and the switch of the third mobot
is near the second mobot.
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
mobot3.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot1.resetToZeroNB();
mobot2.resetToZeroNB();
mobot3.resetToZeroNB();
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();

/* "gun" ready */
mobot3.moveJoint(MOBOT_JOINT2, -90);

/* "gun" scan */
mobot3.moveTo(360, -90, 0, 0);

/* move forward */
mobot1.motionRollForwardNB(360);
mobot2.motionRollForwardNB(360);
mobot3.moveToNB(360, -90, 0, 0);
mobot1.motionWait();
mobot2.motionWait();
mobot3.moveWait();

/* move backward */
mobot1.motionRollForwardNB(-360);
mobot2.motionRollForwardNB(-360);
mobot3.moveToNB(-360, -90, 0, 0);
mobot1.motionWait();
mobot2.motionWait();
mobot3.moveWait();

/* moving the "gun"*/
mobot3.moveTo(0, -45, 45, 0);
mobot3.moveTo(360, -45, 45, 0);
mobot3.moveTo(0, 0, 90, 0);
mobot3.moveTo(360, 0, 90, 0);
mobot3.moveTo(0, 0, 0, 0);
mobot3.moveTo(360, 0, 0, 0);

/* turn right */
mobot1.motionTurnRightNB(180);
mobot2.motionTurnRightNB(180);
mobot3.moveToNB(-90, -90, 0, 0);
mobot1.motionWait();
mobot2.motionWait();
mobot3.moveWait();

/* turn left */
mobot1.motionTurnLeftNB(180);
mobot2.motionTurnLeftNB(180);
mobot3.moveToNB(90, -90, 0, 0);
mobot1.motionWait();
mobot2.motionWait();
mobot3.moveWait();
