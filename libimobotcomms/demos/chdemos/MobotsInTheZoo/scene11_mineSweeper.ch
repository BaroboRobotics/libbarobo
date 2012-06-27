/* Discription:
 * In this scene, three modules are assembled as a mine sweeper.
 *               4
 *            -------
 *             |   |
 *             | 3 |
 *             |   |
 *       3rd  -------
 *             |   |
 *             | 2 |
 *             |   |
 *            -------
 *               1
 *               X
 *     |---------|--------|             
 *1st 1|    2    |   3    | 4    
 *     |---------|--------|             
 *               X
 *     |---------|--------|       
 *2nd 1|    2    |   3    | 4    
 *     |---------|--------|      
 */
 
#include <mobot.h>
CMobot robot1;
CMobot robot2;
CMobot robot3;
int i;
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

robot1.setJointSpeedRatios(0.8, 0.4, 0.4, 0.8);
robot2.setJointSpeedRatios(0.8, 0.4, 0.4, 0.8);
robot3.setJointSpeedRatios(0.25, 0.8, 0.8, 0.8);

/* move forward */
robot1.motionRollForwardNB(360);
robot2.motionRollForwardNB(360);
robot1.motionWait();
robot2.motionWait();

/* scan x axis */
robot3.moveJoint(MOBOT_JOINT1, 90);
robot3.moveJoint(MOBOT_JOINT2, -90);

for(i = -90; i<=90;i = i+ 10)
{
    robot3.moveJointToNB(MOBOT_JOINT2, i);
    robot3.moveWait();
}

robot3.moveJoint(MOBOT_JOINT1, -90);
robot3.moveJoint(MOBOT_JOINT3, 45);

robot1.motionRollForwardNB(360);
robot2.motionRollForwardNB(360);
robot1.motionWait();
robot2.motionWait();
/*
// move backward
robot1.motionRollForwardNB(-360);
robot2.motionRollForwardNB(-360);
robot1.motionWait();
robot2.motionWait();

// turn right
robot1.motionTurnRightNB(90);
robot2.motionTurnRightNB(90);
robot1.motionWait();
robot2.motionWait();

// move forward
robot1.motionRollForwardNB(360);
robot2.motionRollForwardNB(360);
robot1.motionWait();
robot2.motionWait();

// turn left
robot1.motionTurnLeftNB(90);
robot2.motionTurnLeftNB(90);
robot1.motionWait();
robot2.motionWait();
*/
/* scan x axis 
robot3.moveJointTo(MOBOT_JOINT3,0);
robot3.moveJoint(MOBOT_JOINT1, 90);
robot3.moveJoint(MOBOT_JOINT2, -90);

for(i = -90; i<=90;i = i+ 10)
{
    robot3.moveJointToNB(MOBOT_JOINT2, i);
    robot3.moveWait();
}

robot3.moveJoint(MOBOT_JOINT2, 90);
robot3.moveJoint(MOBOT_JOINT1, -90);
*/
