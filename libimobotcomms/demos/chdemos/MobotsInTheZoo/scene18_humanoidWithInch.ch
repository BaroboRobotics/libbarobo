/* Discriptioin:
 * In this scene, four modules are assembled as a humanoid robot with a single robot.
 *           1
 *        -------
 *         |   |
 *         | 2 |
 *         |   |
 *        -------  3rd
 *         |   |
 *         | 3 |
 *         |   |
 *        -------
 *           4
 *           X
 *   ----------------
 *      X         X
 *      1         1
 *   -------   -------
 *    |   |     |   |
 *    | 2 |     | 2 |
 *    |   |     |   |
 *1st-------   ------- 2nd
 *    |   |     |   |
 *    | 3 |     | 3 |
 *    |   |     |   |
 *   -------   -------
 *      4         4
 */
 
#include <mobot.h>
CMobot robot1;
CMobot robot2;
CMobot robot3;
CMobot robot4;
int i;

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

robot1.setJointSpeedRatios(0.35, 0.35, 0.35, 0.35);
robot2.setJointSpeedRatios(0.35, 0.35, 0.35, 0.35);
robot3.setJointSpeedRatios(0.55, 0.55, 0.55, 0.55);
robot4.setJointSpeedRatios(0.55, 0.55, 0.55, 0.55);

/* preparation */
robot1.moveJointToNB(ROBOT_JOINT4, 45);
robot2.moveJointToNB(ROBOT_JOINT4, 45);
robot3.moveJointToNB(ROBOT_JOINT4, -90);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();

// while humanoid is walking forward, robot4 do inchworm left
robot4.motionInchwormLeftNB(5);

/* move forward first step */
robot3.moveJointTo(ROBOT_JOINT3, -90);
robot1.moveTo(0, 45, 45, 45);
robot2.moveTo(0, -45, -45, 45);
robot3.moveJointTo(ROBOT_JOINT3, 90);
robot1.moveTo(0, 0, 0, 45);
robot2.moveTo(0, 45, 45, 45);

for (i = 0; i < 1; i++) {
    /* left leg */
    robot1.moveTo(0, -45, -45, 45);
    robot3.moveJointTo(ROBOT_JOINT3, -90);
    robot2.moveTo(0, 0, 0, 45);
    robot1.moveTo(0, 45, 45, 45);
    /* right leg */
    robot2.moveTo(0, -45, -45, 45);
    robot3.moveJointTo(ROBOT_JOINT3, 90);
    robot1.moveTo(0, 0, 0, 45);
    robot2.moveTo(0, 45, 45, 45);
}

robot2.moveTo(0, 0, 0, 45);
robot4.motionWait();


// while humanoid is walking backward, robot4 do inchworm right
robot4.motionInchwormRightNB(5);

/* move Backward */
robot3.moveJointTo(ROBOT_JOINT3, -90);

for (i = 0; i < 1; i++) {
    /* right leg */
    robot1.moveTo(0, -45, -45, 45);
    robot2.moveTo(0, 45, 45, 45);
    robot3.moveJointTo(ROBOT_JOINT3, 90);
    robot1.moveTo(0, 0, 0, 45);

    /* left leg */
    robot2.moveTo(0, -45, -45, 45);
    robot1.moveTo(0, 45, 45, 45);
    robot3.moveJointTo(ROBOT_JOINT3, -90);
    robot2.moveTo(0, 0, 0, 45);
}

robot1.moveTo(0, 0, 0, 45);
robot4.motionWait();

robot1.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot2.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot3.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);

robot3.moveJointTo(ROBOT_JOINT4, 0);
robot3.moveJointTo(ROBOT_JOINT3, 90);

robot1.moveToNB(0, -90, -90, 45);
robot2.moveToNB(0, -90, -90, 45);
robot1.moveWait();
robot2.moveWait();


robot1.moveToNB(0, -90, 0, 45);
robot2.moveToNB(0, -90, 0, 45);
robot1.moveWait();
robot2.moveWait();

robot1.moveToNB(0, -90, 0, 0);
robot2.moveToNB(0, -90, 0, 0);
robot1.moveWait();
robot2.moveWait();

robot1.moveToNB(0, 0, 90, 0);
robot2.moveToNB(0, 0, 90, 0);
robot3.moveToNB(0, -90, 0, 0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();

robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot3.moveToZeroNB();
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
