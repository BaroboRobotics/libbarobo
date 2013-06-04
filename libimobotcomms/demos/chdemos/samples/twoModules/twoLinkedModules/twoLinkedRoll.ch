/* Filename: twoLinkedRoll.ch
   Control two modules and make them stand simultaneously.
   The joint4 of the first mobot should be connected to the joint1 
   of the second mobot. 
           1st                         2nd
   |---------|--------|       |---------|--------|     
 1 |    2    |    3   | 4 X 1 |    2    |   3    | 4
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
mobot1.resetToZeroNB();
mobot2.resetToZeroNB();
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
