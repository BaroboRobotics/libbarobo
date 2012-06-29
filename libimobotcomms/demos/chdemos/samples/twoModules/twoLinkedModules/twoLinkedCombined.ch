/* Filename: twoLinkedCombined.ch
   Control two modules and make them stand simultaneously and work.
   The joint4 of the first mobot should be connected to the joint1 
   of the second mobot. 
           1st                         2nd
   |---------|--------|       |---------|--------|     
 1 |    2    |    3   | 4 X 1 |    2    |   3    | 4
   |---------|--------|       |---------|--------|
*/
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
int i;

/* Connect mobot variables to the mobot modules. */
mobot1.connect();
mobot2.connect();

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot1.moveToZeroNB();
mobot2.moveToZeroNB();
mobot1.moveWait();
mobot2.moveWait();
/* inch warm right with two modules 1*/
for( i = 0; i < 3; i++){
    mobot2.moveJointTo(MOBOT_JOINT3, 60);
    mobot1.moveJointTo(MOBOT_JOINT2, -60);
    mobot2.moveJointTo(MOBOT_JOINT3, 0);
    mobot1.moveJointTo(MOBOT_JOINT2, 0);
}
/* inch warm left with two modules 1*/
for( i = 0; i < 3; i++){
    mobot1.moveJointTo(MOBOT_JOINT2, -60);
    mobot2.moveJointTo(MOBOT_JOINT3, 60);
    mobot1.moveJointTo(MOBOT_JOINT2, 0);
    mobot2.moveJointTo(MOBOT_JOINT3, 0);
}
/* lifting with two modules */
/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot1.moveToZeroNB();
mobot2.moveToZeroNB();
mobot1.moveWait();
mobot2.moveWait();

/* First time */
mobot1.moveToNB(0, -90, 0, 0);
mobot2.moveToNB(0, 0, 90,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot1.moveToNB(0, -45, 45, 0);
mobot2.moveToNB(0, -45, 45, 0);
mobot1.moveWait();
mobot2.moveWait();
delay(3);
mobot1.moveToNB(0, -90, 0, 0);
mobot2.moveToNB(0, 0, 90,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot1.moveToNB(0, 0, 0, 0);
mobot2.moveToNB(0, 0, 0, 0);
mobot1.moveWait();
mobot2.moveWait();

/* Second lift with 90 degrees*/

mobot1.moveToNB(0, -90, 0, 0);
mobot2.moveToNB(0, 0, 90,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot1.moveToNB(0, 0, 90, 0);
mobot2.moveToNB(0, -90,  0, 0);
mobot1.moveWait();
mobot2.moveWait();
delay(3);
mobot1.moveToNB(0, -90, 0, 0);
mobot2.moveToNB(0, 0, 90,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot1.moveToNB(0, 0, 0, 0);
mobot2.moveToNB(0, 0, 0, 0);
mobot1.moveWait();
mobot2.moveWait();

/* Third time lift with 90 degrees*/
mobot1.moveToNB(0, 0, 90, 0);
mobot2.moveToNB(0, -90,  0, 0);
mobot1.moveWait();
mobot2.moveWait();
delay(3);
/* walk left*/
for(i = 0;i <3; i++){
    mobot1.moveToNB(0, -60, 30, 0);
    mobot1.motionWait();
    mobot2.moveToNB(0, -30, 60, 0);
    mobot2.motionWait();
    mobot1.moveToNB(0, 0, 90, 0);
    mobot1.motionWait();
    mobot2.moveToNB(0, -90,  0, 0);
    mobot2.motionWait();
}

/* walk right*/
for(i = 0;i <3; i++){
    mobot2.moveTo(0, -30, 60, 0);
    mobot2.motionWait();
    mobot1.moveTo(0, -60, 30, 0);
    mobot1.motionWait();

    mobot2.moveToNB(0, -90,  0, 0);
    mobot2.motionWait();
    mobot1.moveToNB(0, 0, 90, 0);
    mobot1.motionWait();
}

mobot1.moveToNB(0,  -90, 0, 0);
mobot2.moveToNB(0, 0, 90,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot1.moveToNB(0, 0, 0, 0);
mobot2.moveToNB(0, 0, 0, 0);
mobot1.moveWait();
mobot2.moveWait();

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */

mobot1.moveToZeroNB();
mobot2.moveToZeroNB();
mobot1.moveWait();
mobot2.moveWait();

/* rolling */

mobot1.motionRollForwardNB(2*360);
mobot2.motionRollForwardNB(2*360);
mobot1.moveWait();
mobot2.moveWait();
mobot1.motionRollBackwardNB(2*360);
mobot2.motionRollBackwardNB(2*360);
mobot1.moveWait();
mobot2.moveWait();
