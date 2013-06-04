/* Filename: dog1.ch
   Control three modules and make them tumble three times in one direction.
        1
     -------
      |   |
     2|   |
      |   |
     ------- 3rd
      |   |
     3|   |
      |   |
     --------
     \  4   /
     /\    /\3
   \/ 4\  /4 \/
   |\  /\/\  /|
2nd| \/3   \/ |  1st
   |  \    /  |2
   |  |2   |  |
  ------  ------
    1        1
All switches are located at the positions with "3" marked.
Before assembling, please make sure each module is in the home position.
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

mobot1.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
mobot2.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
mobot3.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);

mobot1.moveToNB(0, 0, 63.5, 90);
mobot2.moveToNB(0, 0, -63.5, 90);
mobot1.moveWait();
mobot2.moveWait();

for (i = 0; i < 1; i++)
{
    /* mobot 3 on the top*/
    mobot3.moveTo(0, -5, 96, 90);
    mobot1.moveTo(0, -55, 63.5, 90);
    mobot2.moveTo(0, -5, 92.5, 90);

    mobot1.moveToNB(0, 0, -63.5, 90);
    mobot3.moveToNB(0, 0, 63.5, 90);
    mobot1.moveWait();
    mobot3.moveWait();

    /* mobot 2 on the top*/
    mobot2.moveTo(0, -5, 96, 90);
    mobot3.moveTo(0, -55, 63.5, 90);
    mobot1.moveTo(0, -5, 92.5, 90);

    mobot3.moveToNB(0, 0, -63.5, 90);
    mobot2.moveToNB(0, 0, 63.5, 90);
    mobot3.moveWait();
    mobot2.moveWait();

    /* mobot 1 on the top*/
    mobot1.moveTo(0, -5, 96, 90);
    mobot2.moveTo(0, -55, 63.5, 90);
    mobot3.moveTo(0, -5, 92.5, 90);

    mobot2.moveToNB(0, 0, -63.5, 90);
    mobot1.moveToNB(0, 0, 63.5, 90);
    mobot2.moveWait();
    mobot1.moveWait();
}
