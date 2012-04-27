/* Discriptioin:
 * In this scene, two-module robot do as fourWheelDrive.
 *      |---------|--------|             
 *1st 1 |    2    |   3    | 4    
 *      |---------|--------|             
 *                X
 *      |---------|--------|       
 *2nd 1 |    2    |   3    | 4    
 *      |---------|--------|
 *
 *      |---------|--------|       
 *3rd 1 |    2    |   3    | 4    
 *      |---------|--------|
 */
 
#include <mobot.h>
CMobot robot1;
CMobot robot2;
CMobot robot3;

/* Connect robot variables to the robot modules. */
robot1.connect();
robot2.connect();
robot3.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot3.moveToZeroNB();
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();

robot1.setJointSpeedRatios(0.8, 0.5, 0.5, 0.8);
robot2.setJointSpeedRatios(0.8, 0.5, 0.5, 0.8);
robot3.setJointSpeedRatios(0.8, 0.5, 0.5, 0.8);

/* rolling forward*/
robot1.motionRollForwardNB(360);
robot2.motionRollForwardNB(360);
robot3.motionRollForwardNB(360);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
