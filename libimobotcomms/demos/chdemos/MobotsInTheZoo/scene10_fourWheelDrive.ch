/* Discriptioin:
 * In this scene, two-module robot do as fourWheelDrive.
 *      |---------|--------|             
 *1st 1 |    2    |   3    | 4    
 *      |---------|--------|             
 *                X
 *      |---------|--------|       
 *2nd 1 |    2    |   3    | 4    
 *      |---------|--------|      
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

robot1.setJointSpeedRatios(0.8, 0.5, 0.5, 0.8);
robot2.setJointSpeedRatios(0.8, 0.5, 0.5, 0.8);
/* rolling forward*/
robot1.motionRollForwardNB(360);
robot2.motionRollForwardNB(360);

robot1.moveWait();
robot2.moveWait();
/* rolling backward*/
robot1.motionRollForwardNB(-2*360);
robot2.motionRollForwardNB(-2*360);
robot1.moveWait();
robot2.moveWait();
