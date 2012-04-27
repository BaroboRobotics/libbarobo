/* Discription:
 * In this scene, two-linked module do inchworm.
 *           1st                         2nd
 *   |---------|--------|       |---------|--------|     
 * 1 |    2    |   3    | 4 X 1 |    2    |   3    | 4
 *   |---------|--------|       |---------|--------|   
 */
 
#include <mobot.h>
int i;
CMobot robot1;
CMobot robot2;
CMobot robot3;
CMobot robot4;

/* Connect robot variables to the robot modules. */
robot1.connect();
robot2.connect();
robot3.connect();
robot4.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot3.moveToZeroNB();
robot4.moveToZeroNB();
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
robot4.moveWait();

robot3.setJointSpeedRatios(0.05, 0.05, 0.05, 0.05);
robot4.setJointSpeedRatios(0.05, 0.05, 0.05, 0.05);
/* stand up */
robot1.moveToNB(0, 90, 90,  0);
robot2.moveToNB(0,  -90,  -90, 0);
robot1.moveWait();
robot2.moveWait();

robot3.motionRollBackwardNB(360);
robot4.motionRollBackwardNB(360);
/* inworm right */
for( i = 0; i < 10; i++){
    robot2.moveTo(0, -90, -30,  0);
    robot1.moveTo(0,  30,  90, 0);
    robot2.moveTo(0, -90, -90,  0);
    robot1.moveTo(0,  90,  90, 0);
}
robot3.motionWait();
robot4.motionWait();
/* inworm left */
/*
for( i = 0; i < 3; i++){
    robot1.moveTo(0,  30,  90, 0);
    robot2.moveTo(0, -90, -30,  0);
    robot1.moveTo(0,  90,  90, 0);
    robot2.moveTo(0, -90, -90,  0);
}
*/
