/* Filename: twoLinkedInch.ch
   Control two modules and make them stand simultaneously and work.
   The joint4 of the first mobot should be connected to the joint1 
   of the second mobot. 
           1st                         2nd
   |---------|--------|       |---------|--------|     
 1 |    2    |   3    | 4 X 1 |    2    |   3    | 4
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
mobot1.resetToZeroNB();
mobot2.resetToZeroNB();
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
