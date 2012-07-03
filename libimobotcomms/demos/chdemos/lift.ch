/* Filename: lift.ch
   Lift two connected mobots.
   Joint 4 of the 1st mobot should be connected to 
   Joint 1 of the 2nd mobot as
           1st                       2nd
  |---------|--------|      |---------|--------|     
 1|   2     |   3    | 4 X 1|    2    |   3    | 4
  |---------|--------|      |---------|--------|
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

/* First lift */
mobot1.moveToNB(0, -90,  0, 0);
mobot2.moveToNB(0, 0, 90, 0);
mobot1.moveWait();
mobot2.moveWait();
delay(1);

/* Second lift */
mobot1.moveToNB(0, 0, 90,  0);
mobot2.moveToNB(0,  -90, 0, 0);
mobot1.moveWait();
mobot2.moveWait();
delay(1);

/* Move to zero position */
mobot1.moveToZeroNB(); 
mobot2.moveToZeroNB();
mobot1.moveWait();
mobot2.moveWait();
