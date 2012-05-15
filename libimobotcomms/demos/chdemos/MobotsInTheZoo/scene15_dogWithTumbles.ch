/* Discription:
 * In this scene, five modules are assembled as a three-module dog with two single robots.
 *              1
 *           -------
 *            |   |
 *           2|   |
 *            |   |
 *           ------- 3rd
 *            |   |
 *           3|   |
 *            |   |
 *           --------
 *           \  4   /
 *           /\    /\3
 *         \/ 4\  /4 \/
 *         |\  /\/\  /|
 *      2nd| \/3   \/ |  1st
 *         |  \    /  |2
 *         |  |2   |  |
 *        ------  ------
 *          1        1
 *
 *      |---------|--------|
 *4th 1 |    2    |   3    | 4
 *      |---------|--------|
 *                X
 *      |---------|--------|
 *5th 1 |    2    |   3    | 4
 *      |---------|--------|
 *
 *All switches are located at the positions with "3" marked.
 *Before assembling, please make sure every modules are in the home position.
 */
 
#include <mobot.h>
CMobot robot1;
CMobot robot2;
CMobot robot3;
CMobot robot4;
CMobot robot5;
int i;

/* Connect robot variables to the robot modules. */
robot1.connect();
robot2.connect();
robot3.connect();
robot4.connect();
robot5.connect();

// let robot 4 and 5 move to zero
robot4.moveToZeroNB();
robot5.moveToZeroNB();
robot4.moveWait();
robot5.moveWait();

// set speeds of robots
robot1.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
robot2.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
robot3.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
robot4.setJointSpeedRatios(0.8, 0.8, 0.8, 0.8);
robot5.setJointSpeedRatios(0.8, 0.8, 0.8, 0.8);

robot1.moveToNB(0, 0, 63.5, 90);
robot2.moveToNB(0, 0, -63.5, 90);
robot1.moveWait();
robot2.moveWait();
// direction: right
robot4.motionTumbleRightNB(2);
robot5.motionTumbleRightNB(2);

for (i = 0; i < 1; i++)
{
    /* robot 3 on the top*/
    robot3.moveTo(0, -5, 92, 90);
    robot1.moveTo(0, -55, 63.5, 90);
    robot2.moveTo(0, -5, 92.5, 90);
    sleep(1);
    robot1.moveToNB(0, 0, -63.5, 90);
    robot3.moveToNB(0, 0, 63.5, 90);
    robot1.moveWait();
    robot3.moveWait();

    /* robot 2 on the top */
    robot2.moveTo(0, -5, 92, 90);
    robot3.moveTo(0, -55, 63.5, 90);
    robot1.moveTo(0, -5, 92.5, 90);

    robot3.moveToNB(0, 0, -63.5, 90);
    robot2.moveToNB(0, 0, 63.5, 90);
    robot3.moveWait();
    robot2.moveWait();

    /* robot 1 on the top
    robot1.moveTo(0, -5, 96, 90);
    robot2.moveTo(0, -55, 63.5, 90);
    robot3.moveTo(0, -5, 92.5, 90);

    robot2.moveToNB(0, 0, -63.5, 90);
    robot1.moveToNB(0, 0, 63.5, 90);
    robot2.moveWait();
    robot1.moveWait();
    */
}
robot4.motionWait();
robot5.motionWait();

// direction: left
//robot4.motionTumbleLeftNB(1);
//robot5.motionTumbleLeftNB(2);

for (i = 0; i < 1; i++) {
    /* robot 3 on the top 
    robot3.moveTo(0, 5, -96, 90);
    robot2.moveTo(0, 55, -63.5, 90);
    robot1.moveTo(0, 5, -92.5, 90);

    robot2.moveToNB(0, 0, 63.5, 90);
    robot3.moveToNB(0, 0, -63.5, 90);
    robot2.moveWait();
    robot3.moveWait();
*/
    /* robot 1 on the top 
    robot1.moveTo(0, 5, -96, 90);
    robot3.moveTo(0, 55, -63.5, 90);
    robot2.moveTo(0, 5, -92.5, 90);

    robot3.moveToNB(0, 0, 63.5, 90);
    robot1.moveToNB(0, 0, -63.5, 90);
    robot3.moveWait();
    robot1.moveWait();
*/
    /* robot 2 on the top
    robot2.moveTo(0, 5, -96, 90);
    robot1.moveTo(0, 55, -63.5, 90);
    robot3.moveTo(0, 5, -92.5, 90);
sleep(1);
    robot1.moveToNB(0, 0, 63.5, 90);
    robot2.moveToNB(0, 0, -63.5, 90);
    robot1.moveWait();
    robot2.moveWait();*/
}
//robot4.motionWait();
//robot5.motionWait();
