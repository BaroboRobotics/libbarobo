/* Filename: fourWheelPassive.ch
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
int i;
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

/* rolling with second one passive */
robot1.motionRollForwardNB(2*360);
robot2.moveContinuousTime(ROBOT_NEUTRAL, ROBOT_HOLD, 
                        ROBOT_HOLD, ROBOT_NEUTRAL, 5000);
robot1.moveWait();
robot2.moveWait();
