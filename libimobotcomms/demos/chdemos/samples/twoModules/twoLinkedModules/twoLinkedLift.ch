/* Filename: twoLinkedLift.ch
   Control two modules and make them stand simultaneously.
   The joint4 of the first mobot should be connected to the joint1 
   of the second mobot. 
           1st                         2nd
   |---------|--------|         |---------|--------|     
 1 |    2    |   3    | 4 <=> 1 |    2    |   3    | 4
   |---------|--------|         |---------|--------|   
*/

#include <mobot.h>
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

/* First time */

mobot1.moveToNB(0,  -90, 0, 0);
mobot2.moveToNB(0, 0, 90,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot1.moveToNB(0, -45,  45, 0);
mobot2.moveToNB(0, -45,  45, 0);
mobot1.moveWait();
mobot2.moveWait();
delay(5);
mobot1.moveToNB(0,  -90, 0, 0);
mobot2.moveToNB(0, 0, 90,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot1.moveToNB(0, 0, 0, 0);
mobot2.moveToNB(0, 0, 0, 0);
mobot1.moveWait();
mobot2.moveWait();

/* Second lift with 90 degrees*/
mobot1.moveToNB(0,  -90, 0, 0);
mobot2.moveToNB(0, 0, 90,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot1.moveToNB(0, 0, 90, 0);
mobot2.moveToNB(0, -90,  0, 0);
mobot1.moveWait();
mobot2.moveWait();
delay(5);
mobot1.moveToNB(0,  -90, 0, 0);
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
delay(5);
mobot1.moveToNB(0,  -90, 0, 0);
mobot2.moveToNB(0, 0, 90,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot1.moveToNB(0, 0, 0, 0);
mobot2.moveToNB(0, 0, 0, 0);
mobot1.moveWait();
mobot2.moveWait();
