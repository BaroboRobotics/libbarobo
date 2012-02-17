/* Filename: twoLinkedLift.ch
   Control two modules and make them stand simultaneously.
   The joint4 of the first robot should be connected to the joint1 
   of the second robot. 
           1st                         2nd
   |---------|--------|         |---------|--------|     
 1 |    2    |   3    | 4 <=> 1 |    2    |   3    | 4
   |---------|--------|         |---------|--------|   
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

/* First time */

robot1.moveToNB(0,  -90, 0, 0);
robot2.moveToNB(0, 0, 90,  0);
robot1.moveWait();
robot2.moveWait();
robot1.moveToNB(0, -45,  45, 0);
robot2.moveToNB(0, -45,  45, 0);
robot1.moveWait();
robot2.moveWait();
sleep(5);
robot1.moveToNB(0,  -90, 0, 0);
robot2.moveToNB(0, 0, 90,  0);
robot1.moveWait();
robot2.moveWait();
robot1.moveToNB(0, 0, 0, 0);
robot2.moveToNB(0, 0, 0, 0);
robot1.moveWait();
robot2.moveWait();

/* Second lift with 90 degrees*/
robot1.moveToNB(0,  -90, 0, 0);
robot2.moveToNB(0, 0, 90,  0);
robot1.moveWait();
robot2.moveWait();
robot1.moveToNB(0, 0, 90, 0);
robot2.moveToNB(0, -90,  0, 0);
robot1.moveWait();
robot2.moveWait();
sleep(5);
robot1.moveToNB(0,  -90, 0, 0);
robot2.moveToNB(0, 0, 90,  0);
robot1.moveWait();
robot2.moveWait();
robot1.moveToNB(0, 0, 0, 0);
robot2.moveToNB(0, 0, 0, 0);
robot1.moveWait();
robot2.moveWait();

/* Third time lift with 90 degrees*/
robot1.moveToNB(0, 0, 90, 0);
robot2.moveToNB(0, -90,  0, 0);
robot1.moveWait();
robot2.moveWait();
sleep(5);
robot1.moveToNB(0,  -90, 0, 0);
robot2.moveToNB(0, 0, 90,  0);
robot1.moveWait();
robot2.moveWait();
robot1.moveToNB(0, 0, 0, 0);
robot2.moveToNB(0, 0, 0, 0);
robot1.moveWait();
robot2.moveWait();
