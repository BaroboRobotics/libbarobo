/* Filename: fourWheelDrive.ch
   Control two modules with four wheels.
     |---------|--------|             
1st 1|    2    |   3    | 4    
     |---------|--------|             
               X
     |---------|--------|       
2nd 1|    2    |   3    | 4    
     |---------|--------|      
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

/* rolling forward*/
robot1.motionRollForwardNB(2*360);
robot2.motionRollForwardNB(2*360);
robot1.moveWait();
robot2.moveWait();

/* rolling backward*/
robot1.motionRollForwardNB(-2*360);
robot2.motionRollForwardNB(-2*360);
robot1.moveWait();
robot2.moveWait();
