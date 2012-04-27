/* Discriptioin:
 * In this scene, one module do as twoWheelDrive.
 *   |                      |
 *   | |---------|--------| |            
 *   |1|    2    |   3    |4|    
 *   | |---------|--------| |
 *   |        -------       |
 *               |
 *Joint 2 and joint 3 are fixed by a connector with a metal caster.
 *Joint 1 and joint 4 have wheels attached. 
 */
#include <mobot.h>
CMobot robot;

/* Connect robot variables to the robot modules. */
robot.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

robot.setJointSpeedRatios(0.4, 0.5, 0.5, 0.4);

/* rolling forward*/
robot.motionRollForward(360);

/* rolling backward*/
robot.motionRollBackward(360);

/* turn right and left */
robot.motionTurnRight(360);
robot.motionTurnLeft(360);
