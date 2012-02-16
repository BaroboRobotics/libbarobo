/* Filename: combined.ch
   Control two modules and make them stand simultaneously and work.
   The joint4 of the first robot should be connected to the joint1 
   of the second robot. 
           1st                         2nd
   |---------|--------|       |---------|--------|     
 1 |    2    |    3   | 4 X 1 |    2    |   3    | 4
   |---------|--------|       |---------|--------|
*/
#include <mobot.h>
CMobot robot1;
CMobot robot2;
int i;

/* Connect robot variables to the robot modules. */
robot1.connect();
robot2.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot1.moveWait();
robot2.moveWait();
/* inch warm right with two modules 1*/
for( i = 0; i < 3; i++){
    robot2.moveJointTo(ROBOT_JOINT3, 60);
    robot1.moveJointTo(ROBOT_JOINT2, -60);
    robot2.moveJointTo(ROBOT_JOINT3, 0);
    robot1.moveJointTo(ROBOT_JOINT2, 0);
}
/* inch warm left with two modules 1*/
for( i = 0; i < 3; i++){
    robot1.moveJointTo(ROBOT_JOINT2, -60);
    robot2.moveJointTo(ROBOT_JOINT3, 60);
    robot1.moveJointTo(ROBOT_JOINT2, 0);
    robot2.moveJointTo(ROBOT_JOINT3, 0);
}
/* lifting with two modules */
/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot1.moveWait();
robot2.moveWait();

/* First time */
robot1.moveToNB(0, -90, 0, 0);
robot2.moveToNB(0, 0, 90,  0);
robot1.moveWait();
robot2.moveWait();
robot1.moveToNB(0, -45, 45, 0);
robot2.moveToNB(0, -45, 45, 0);
robot1.moveWait();
robot2.moveWait();
sleep(3);
robot1.moveToNB(0, -90, 0, 0);
robot2.moveToNB(0, 0, 90,  0);
robot1.moveWait();
robot2.moveWait();
robot1.moveToNB(0, 0, 0, 0);
robot2.moveToNB(0, 0, 0, 0);
robot1.moveWait();
robot2.moveWait();

/* Second lift with 90 degrees*/

robot1.moveToNB(0, -90, 0, 0);
robot2.moveToNB(0, 0, 90,  0);
robot1.moveWait();
robot2.moveWait();
robot1.moveToNB(0, 0, 90, 0);
robot2.moveToNB(0, -90,  0, 0);
robot1.moveWait();
robot2.moveWait();
sleep(3);
robot1.moveToNB(0, -90, 0, 0);
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
sleep(3);
/* walk left*/
for(i = 0;i <3; i++){
    robot1.moveToNB(0, -60, 30, 0);
    robot1.motionWait();
    robot2.moveToNB(0, -30, 60, 0);
    robot2.motionWait();
    robot1.moveToNB(0, 0, 90, 0);
    robot1.motionWait();
    robot2.moveToNB(0, -90,  0, 0);
    robot2.motionWait();
}

/* walk right*/
for(i = 0;i <3; i++){
    robot2.moveTo(0, -30, 60, 0);
    robot2.motionWait();
    robot1.moveTo(0, -60, 30, 0);
    robot1.motionWait();

    robot2.moveToNB(0, -90,  0, 0);
    robot2.motionWait();
    robot1.moveToNB(0, 0, 90, 0);
    robot1.motionWait();
}

robot1.moveToNB(0,  -90, 0, 0);
robot2.moveToNB(0, 0, 90,  0);
robot1.moveWait();
robot2.moveWait();
robot1.moveToNB(0, 0, 0, 0);
robot2.moveToNB(0, 0, 0, 0);
robot1.moveWait();
robot2.moveWait();

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
