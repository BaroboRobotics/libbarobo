/* Discription:
 * In this scene, three modules tumble as a dog.
 *        1
 *     -------
 *      |   |
 *     2|   |
 *      |   |
 *     ------- 3rd
 *      |   |
 *     3|   |
 *      |   |
 *     --------
 *     \  4   /
 *     /\    /\3
 *   \/ 4\  /4 \/
 *   |\  /\/\  /|
 *2nd| \/3   \/ |  1st
 *   |  \    /  |2
 *   |  |2   |  |
 *  ------  ------
 *    1        1
 *All switches are located at the positions with "3" marked.
 *Before assembling, please make sure each module is in the home position.
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

robot1.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
robot2.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
robot3.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);

robot1.moveToNB(0, 0, 63.5, 90);
robot2.moveToNB(0, 0, -63.5, 90);
robot1.moveWait();
robot2.moveWait();

for (i = 0; i < 1; i++)
{
    /* robot 3 on the top*/
    robot3.moveTo(0, -5, 92, 90);
    robot1.moveTo(0, -55, 63.5, 90);
    robot2.moveTo(0, -5, 92.5, 90);

    robot1.moveToNB(0, 0, -63.5, 90);
    robot3.moveToNB(0, 0, 63.5, 90);
    robot1.moveWait();
    robot3.moveWait();

    /* robot 2 on the top*/
    robot2.moveTo(0, -5, 92, 90);
    robot3.moveTo(0, -55, 63.5, 90);
    robot1.moveTo(0, -5, 92.5, 90);

    robot3.moveToNB(0, 0, -63.5, 90);
    robot2.moveToNB(0, 0, 63.5, 90);
    robot3.moveWait();
    robot2.moveWait();

    /* robot 1 on the top*/
    robot1.moveTo(0, -5, 92, 90);
    robot2.moveTo(0, -55, 63.5, 90);
    robot3.moveTo(0, -5, 92.5, 90);

    robot2.moveToNB(0, 0, -63.5, 90);
    robot1.moveToNB(0, 0, 63.5, 90);
    robot2.moveWait();
    robot1.moveWait();
}
