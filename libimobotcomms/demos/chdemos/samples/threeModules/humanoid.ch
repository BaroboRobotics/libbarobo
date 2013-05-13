/* Filename: humanoid.ch
   Control three modules as a humanoid mobot.
           1
        -------
         |   |
         | 2 |
         |   |
        -------  3rd
         |   |
         | 3 |
         |   |
        -------
           4
           X
   ----------------
      X         X
      1         1
   -------   -------
    |   |     |   |
    | 2 |     | 2 |
    |   |     |   |
1st-------   ------- 2nd
    |   |     |   |
    | 3 |     | 3 |
    |   |     |   |
   -------   -------
      4         4
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

mobot1.setJointSpeedRatios(0.55, 0.55, 0.55, 0.55);
mobot2.setJointSpeedRatios(0.55, 0.55, 0.55, 0.55);
mobot3.setJointSpeedRatios(0.55, 0.55, 0.55, 0.55);

/* preparation */
mobot1.moveJointToNB(ROBOT_JOINT4, 45);
mobot2.moveJointToNB(ROBOT_JOINT4, 45);
mobot3.moveJointToNB(ROBOT_JOINT4, -90);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();

/* first step */
mobot3.moveJointTo(ROBOT_JOINT3, -90);
mobot1.moveTo(0, 45, 45, 45);
mobot2.moveTo(0, -45, -45, 45);
mobot3.moveJointTo(ROBOT_JOINT3, 90);
mobot1.moveTo(0, 0, 0, 45);
mobot2.moveTo(0, 45, 45, 45);

for (i = 0; i < 1; i++) {
    /* left leg */
    mobot1.moveTo(0, -45, -45, 45);
    mobot3.moveJointTo(ROBOT_JOINT3, -90);
    mobot2.moveTo(0, 0, 0, 45);
    mobot1.moveTo(0, 45, 45, 45);
    /* right leg */
    mobot2.moveTo(0, -45, -45, 45);
    mobot3.moveJointTo(ROBOT_JOINT3, 90);
    mobot1.moveTo(0, 0, 0, 45);
    mobot2.moveTo(0, 45, 45, 45);
}

mobot2.moveTo(0, 0, 0, 45);

/* move Backward */
mobot3.moveJointTo(ROBOT_JOINT3, -90);

for (i = 0; i < 1; i++) {
    /* right leg */
    mobot1.moveTo(0, -45, -45, 45);
    mobot2.moveTo(0, 45, 45, 45);
    mobot3.moveJointTo(ROBOT_JOINT3, 90);
    mobot1.moveTo(0, 0, 0, 45);

    /* left leg */
    mobot2.moveTo(0, -45, -45, 45);
    mobot1.moveTo(0, 45, 45, 45);
    mobot3.moveJointTo(ROBOT_JOINT3, -90);
    mobot2.moveTo(0, 0, 0, 45);
}

mobot1.moveTo(0, 0, 0, 45);

mobot1.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
mobot2.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
mobot3.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);

mobot3.moveJointTo(ROBOT_JOINT4, 0);
mobot3.moveJointTo(ROBOT_JOINT3, 90);

mobot1.moveToNB(0, -90, -90, 45);
mobot2.moveToNB(0, -90, -90, 45);
mobot1.moveWait();
mobot2.moveWait();


mobot1.moveToNB(0, -90, 0, 45);
mobot2.moveToNB(0, -90, 0, 45);
mobot1.moveWait();
mobot2.moveWait();

mobot1.moveToNB(0, -90, 0, 0);
mobot2.moveToNB(0, -90, 0, 0);
mobot1.moveWait();
mobot2.moveWait();

mobot1.moveToNB(0, 0, 90, 0);
mobot2.moveToNB(0, 0, 90, 0);
mobot3.moveToNB(0, -90, 0, 0);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();

mobot1.resetToZeroNB();
mobot2.resetToZeroNB();
mobot3.resetToZeroNB();
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
