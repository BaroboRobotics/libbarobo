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

/* set robots speed */
robot1.setJointSpeedRatios(0.35, 0.35, 0.35, 0.35);
robot2.setJointSpeedRatios(0.35, 0.35, 0.35, 0.35);
robot3.setJointSpeedRatios(0.35, 0.35, 0.35, 0.35);
robot4.setJointSpeedRatios(0.35, 0.35, 0.35, 0.35);

/* tripleLinked one First motion */
robot1.moveJointNB(MOBOT_JOINT4, 90);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveJointNB(MOBOT_JOINT1, 90);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();

/* tripleLinked one Second motion */
robot1.moveToNB(0,  -90, 0, 90);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveToNB(90, 0, 90,  0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
delay(1);

/* tripleLinked one third motion */
robot1.moveToNB(0, 0, 90, 90);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveToNB(90, -90, 0,  0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
delay(1);
/* tripleLinked one Fourth motion */
robot1.moveJointNB(MOBOT_JOINT4, -90);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveJointNB(MOBOT_JOINT1, -90);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
delay(1);

robot1.moveToNB(0, 0, 90, 0);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveToNB(0, -90, 0,  0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
delay(1);

/*single module one */
robot4.motionRollForward(1.5*360);
robot4.motionRollBackward(1.5*360);
robot.motionInchwormLeft(2);
robot.motionInchwormRight(2);

/* tenth motion */
robot1.moveToNB(0, 0, 90, 0);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveToNB(0, -90, 0,  0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
delay(1);

/* 12th motion */
robot1.moveToNB(0,  -90, 0, 90);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveToNB(90, 0, 90,  0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
delay(2);

/* 14th motion */
robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot3.moveToZeroNB();
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
