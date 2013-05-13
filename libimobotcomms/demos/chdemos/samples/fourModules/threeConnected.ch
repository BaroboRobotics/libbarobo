/* Filename: threeConnected.ch
   Control four modules. Three of them are connected.
          1st                       2nd                         3rd
 |---------|--------|       |---------|--------|       |---------|--------|   
1|    2    |   3    | 4 X 1 |    2    |   3    | 4 X 1 |    2    |   3    | 4 
 |---------|--------|       |---------|--------|       |---------|--------|  
 
                                     4th
                            |---------|--------| 
                           1|    2    |   3    |4  
                            |---------|--------| 
                            
NOTE: the surface should be smooth in order for all motions of
      this program to work properly. If the surface is not smooth,
       you may extract a portion of this program to work.
*/
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
CMobot mobot3;
CMobot mobot4;

/* Connect mobot variables to the mobot modules. */
mobot1.connect();
mobot2.connect();
mobot3.connect();
mobot4.connect();

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot1.resetToZeroNB();
mobot2.resetToZeroNB();
mobot3.resetToZeroNB();
mobot4.resetToZeroNB();
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
mobot4.moveWait();

/* set mobots speed */
mobot1.setJointSpeedRatios(0.35, 0.35, 0.35, 0.35);
mobot2.setJointSpeedRatios(0.35, 0.35, 0.35, 0.35);
mobot3.setJointSpeedRatios(0.35, 0.35, 0.35, 0.35);
mobot4.setJointSpeedRatios(0.35, 0.35, 0.35, 0.35);

/* tripleLinked one First motion */
mobot1.moveJointNB(ROBOT_JOINT4, 90);
mobot2.moveToNB(0, 0, 0, 0);
mobot3.moveJointNB(ROBOT_JOINT1, 90);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();

/* tripleLinked one Second motion */
mobot1.moveToNB(0,  -90, 0, 90);
mobot2.moveToNB(0, 0, 0, 0);
mobot3.moveToNB(90, 0, 90,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
delay(1);

/* tripleLinked one third motion */
mobot1.moveToNB(0, 0, 90, 90);
mobot2.moveToNB(0, 0, 0, 0);
mobot3.moveToNB(90, -90, 0,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
delay(1);
/* tripleLinked one Fourth motion */
mobot1.moveJointNB(ROBOT_JOINT4, -90);
mobot2.moveToNB(0, 0, 0, 0);
mobot3.moveJointNB(ROBOT_JOINT1, -90);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
delay(1);

mobot1.moveToNB(0, 0, 90, 0);
mobot2.moveToNB(0, 0, 0, 0);
mobot3.moveToNB(0, -90, 0,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
delay(1);

/*single module one */
mobot4.motionRollForward(1.5*360);
mobot4.motionRollBackward(1.5*360);
mobot.motionInchwormLeft(2);
mobot.motionInchwormRight(2);

/* tenth motion */
mobot1.moveToNB(0, 0, 90, 0);
mobot2.moveToNB(0, 0, 0, 0);
mobot3.moveToNB(0, -90, 0,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
delay(1);

/* 12th motion */
mobot1.moveToNB(0,  -90, 0, 90);
mobot2.moveToNB(0, 0, 0, 0);
mobot3.moveToNB(90, 0, 90,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
delay(2);

/* 14th motion */
mobot1.resetToZeroNB();
mobot2.resetToZeroNB();
mobot3.resetToZeroNB();
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
