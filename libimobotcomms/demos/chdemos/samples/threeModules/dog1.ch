/* Filename: dog1.ch
   Control three modules and make them tumble three times in one direction.
        4
     -------
      |   |
     3|   |
      |   |
     ------- 3rd
      |   |
     2|   |
      |   |
     --------
     \  1   /
     /\    /\2
   \/ 1\  /1 \/
   |\  /\/\  /|
2nd| \/2   \/ |  1st
   |  \    /  |3
   |  |3   |  |
  ------  ------
    4        4
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
    robot3.moveTo(0, -5, 96, 90);
    robot1.moveTo(0, -55, 63.5, 90);
    robot2.moveTo(0, -5, 92.5, 90);

    robot1.moveToNB(0, 0, -63.5, 90);
    robot3.moveToNB(0, 0, 63.5, 90);
    robot1.moveWait();
    robot3.moveWait();

    /* robot 2 on the top*/
    robot2.moveTo(0, -5, 96, 90);
    robot3.moveTo(0, -55, 63.5, 90);
    robot1.moveTo(0, -5, 92.5, 90);

    robot3.moveToNB(0, 0, -63.5, 90);
    robot2.moveToNB(0, 0, 63.5, 90);
    robot3.moveWait();
    robot2.moveWait();

    /* robot 1 on the top*/
    robot1.moveTo(0, -5, 96, 90);
    robot2.moveTo(0, -55, 63.5, 90);
    robot3.moveTo(0, -5, 92.5, 90);

    robot2.moveToNB(0, 0, -63.5, 90);
    robot1.moveToNB(0, 0, 63.5, 90);
    robot2.moveWait();
    robot1.moveWait();
}
