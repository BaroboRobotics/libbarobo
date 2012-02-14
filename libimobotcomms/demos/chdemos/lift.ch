/* Filename: lift.ch
   Lift two connected robots.
   Joint 4 of the 1st robot should be connected to 
   Joint 1 of the 2nd robot as
           1st                       2nd
  |---------|--------|      |---------|--------|     
 1|   2     |   3    | 4 X 1|    2    |   3    | 4
  |---------|--------|      |---------|--------|
*/
#include <mobot.h>
CMobot robot1;
CMobot robot2;

/* Connect robot variables to the robot modules. */
robot1.connect();
robot2.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot1.moveToZeroNB();
robot2.moveToZeroNB();

robot1.moveWait();
robot2.moveWait();

/* First lift */
robot1.moveNB(0, -90,  0, 0);
robot2.moveNB(0, 0, 90, 0);
robot1.moveWait();
robot2.moveWait();
/* Second lift */
robot1.moveToNB(0, 0, 90,  0);
robot2.moveToNB(0,  -90, 0, 0);
robot1.moveWait();
robot2.moveWait();
/* Move to zero position */
robot1.moveToZeroNB(); 
robot2.moveToZeroNB();
robot1.moveWait();
robot2.moveWait();
