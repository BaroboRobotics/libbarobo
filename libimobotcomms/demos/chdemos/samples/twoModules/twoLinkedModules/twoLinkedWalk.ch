/* Filename: twoLinkedWalk.ch
   Control two modules and make them stand simultaneously.
   The joint4 of the first mobot should be connected to the joint1 
   of the second mobot. 
           1st                         2nd
   |---------|--------|       |---------|--------|     
 1 |    2    |   3    | 4 X 1 |    2    |   3    | 4
   |---------|--------|       |---------|--------|   
*/
#include <mobot.h>
int i;
CMobot mobot1;
CMobot mobot2;

/* Connect mobot variables to the mobot modules. */
mobot1.connect();
mobot2.connect();

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot1.moveToZeroNB();
mobot2.moveToZeroNB();
mobot1.moveWait();
mobot2.moveWait();

/* lift */
mobot1.moveToNB(0, 0, 90, 0);
mobot2.moveToNB(0, -90,  0, 0);
mobot1.moveWait();
mobot2.moveWait();

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
