/* Filename: lift.ch
   Control two modules and make them stand simultaneously.
   The joint4 of the first robot should be connected to the joint1 
   of the second robot. 
           1st                         2nd
   |---------|--------|       |---------|--------|     
 1 |    2    |    3   | 4 X 1 |    2    |   3    | 4
   |---------|--------|       |---------|--------|   
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

/* rolling */
robot1.motionRollForwardNB(2*360);
robot2.motionRollForwardNB(2*360);
robot1.moveWait();
robot2.moveWait();

robot1.motionRollBackwardNB(2*360);
robot2.motionRollBackwardNB(2*360);
robot1.moveWait();
robot2.moveWait();
/*
robot1.moveContinuousNB(ROBOT_JOINT_FORWARD, ROBOT_JOINT_HOLD, 
                        ROBOT_JOINT_HOLD, ROBOT_JOINT_FORWARD);
robot2.moveContinuousNB(ROBOT_JOINT_FORWARD, ROBOT_JOINT_HOLD, 
                        ROBOT_JOINT_HOLD, ROBOT_JOINT_FORWARD);
robot1.moveWait();
robot2.moveWait();
*/

