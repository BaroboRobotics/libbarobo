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
CMobot mobot1;
CMobot mobot2;
CMobot mobot3;
int i;
/* Connect mobot variables to the mobot modules. */
mobot1.connect();
mobot2.connect();
mobot3.connect();

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot1.resetToZeroNB();
mobot2.resetToZeroNB();
mobot3.resetToZeroNB();
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();

mobot1.setJointSpeedRatios(0.8, 0.4, 0.4, 0.8);
mobot2.setJointSpeedRatios(0.8, 0.4, 0.4, 0.8);
mobot3.setJointSpeedRatios(0.25, 0.8, 0.8, 0.8);

/* move forward */
mobot1.motionRollForwardNB(360);
mobot2.motionRollForwardNB(360);
mobot1.motionWait();
mobot2.motionWait();

/* scan x axis */
mobot3.moveJoint(ROBOT_JOINT1, 90);
mobot3.moveJoint(ROBOT_JOINT2, -90);

for(i = -90; i<=90;i = i+ 10)
{
    mobot3.moveJointToNB(ROBOT_JOINT2, i);
    mobot3.moveWait();
}

mobot3.moveJoint(ROBOT_JOINT1, -90);
mobot3.moveJoint(ROBOT_JOINT3, 45);

mobot1.motionRollForwardNB(360);
mobot2.motionRollForwardNB(360);
mobot1.motionWait();
mobot2.motionWait();
/*
// move backward
mobot1.motionRollForwardNB(-360);
mobot2.motionRollForwardNB(-360);
mobot1.motionWait();
mobot2.motionWait();

// turn right
mobot1.motionTurnRightNB(90);
mobot2.motionTurnRightNB(90);
mobot1.motionWait();
mobot2.motionWait();

// move forward
mobot1.motionRollForwardNB(360);
mobot2.motionRollForwardNB(360);
mobot1.motionWait();
mobot2.motionWait();

// turn left
mobot1.motionTurnLeftNB(90);
mobot2.motionTurnLeftNB(90);
mobot1.motionWait();
mobot2.motionWait();
*/
/* scan x axis 
mobot3.moveJointTo(ROBOT_JOINT3,0);
mobot3.moveJoint(ROBOT_JOINT1, 90);
mobot3.moveJoint(ROBOT_JOINT2, -90);

for(i = -90; i<=90;i = i+ 10)
{
    mobot3.moveJointToNB(ROBOT_JOINT2, i);
    mobot3.moveWait();
}

mobot3.moveJoint(ROBOT_JOINT2, 90);
mobot3.moveJoint(ROBOT_JOINT1, -90);
*/
