/* Filename: tripleLift.ch
   Control three modules.
   The joint4 of the first robot should be connected to the joint1 
   of the second robot. 
          1st                      2nd                         3rd
 |---------|--------|       |---------|--------|       |---------|--------|   
1|    2    |   3    | 4 X 1 |    2    |   3    | 4 X 1 |    2    |   3    | 4 
 |---------|--------|       |---------|--------|       |---------|--------|   
*/
#include <mobot.h>
CMobot robot1;
CMobot robot2;
CMobot robot3;

/* Connect robot variables to the robot modules. */
robot1.connect();
robot2.connect();
robot3.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot3.moveToZeroNB();
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();

robot1.setJointSpeedRatio(ROBOT_JOINT1, 0.1);
//robot1.setJointSpeedRatio(ROBOT_JOINT2, 0.3);
//robot1.setJointSpeedRatio(ROBOT_JOINT3, 0.3);
robot1.setJointSpeedRatio(ROBOT_JOINT4, 0.1);
//robot2.setJointSpeedRatio(ROBOT_JOINT1, 0.3);
//robot2.setJointSpeedRatio(ROBOT_JOINT2, 0.3);
//robot2.setJointSpeedRatio(ROBOT_JOINT3, 0.3);
//robot2.setJointSpeedRatio(ROBOT_JOINT4, 0.3);
robot3.setJointSpeedRatio(ROBOT_JOINT1, 0.1);
//robot3.setJointSpeedRatio(ROBOT_JOINT2, 0.3);
//robot3.setJointSpeedRatio(ROBOT_JOINT3, 0.3);
robot3.setJointSpeedRatio(ROBOT_JOINT4, 0.1);

/* First motion */
robot1.moveJointNB(ROBOT_JOINT4, 90);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveJointNB(ROBOT_JOINT1, 90);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();

/* Second motion */
robot1.moveToNB(0,  -90, 0, 90);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveToNB(90, 0, 90,  0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();

sleep(2);

/* third motion */
robot1.moveToNB(0, 0, 90, 90);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveToNB(90, -90, 0,  0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();

sleep(2);

/* fourth motion: spin the middle one */
robot1.moveJointNB(ROBOT_JOINT4, 270);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveJointNB(ROBOT_JOINT1, 270);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();

sleep(2);

/* fifth motion */
robot1.moveToNB(0, 45, 90, 0);
robot2.moveToNB(0, -45, 45, 0);
robot3.moveToNB(0, -90, -45,  0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();

sleep(2);

/* sixth motion */
robot1.moveToNB(45, 45, 90, 45);
robot2.moveToNB(0, -45, 45, 0);
robot3.moveToNB(45, -90, -45,  45);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
sleep(2);

/* seventh motion */
robot1.moveToNB(0, 45, 90, 0);
robot2.moveToNB(0, -45, 45, 0);
robot3.moveToNB(0, -90, -45,  0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
sleep(2);

/* eighth motion */
robot1.moveToNB(-45, 45, 90, -45);
robot2.moveToNB(0, -45, 45, 0);
robot3.moveToNB(-45, -90, -45,  -45);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
sleep(2);

/* ninth motion */
robot1.moveToNB(0, 45, 90, 0);
robot2.moveToNB(0, -45, 45, 0);
robot3.moveToNB(0, -90, -45,  0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
sleep(2);

/* tenth motion */
robot1.moveToNB(0, 0, 90, 0);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveToNB(0, -90, 0,  0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
sleep(2);

/* 11th motion */
robot1.moveJointNB(ROBOT_JOINT4, 90);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveJointNB(ROBOT_JOINT1, 90);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
sleep(2);

/* 12th motion */
robot1.moveToNB(0,  -90, 0, 90);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveToNB(90, 0, 90,  0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
sleep(2);

/* 13th motion */
robot1.moveJointNB(ROBOT_JOINT4, -90);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveJointNB(ROBOT_JOINT1, -90);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
sleep(2);

/* 14th motion */
robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot3.moveToZeroNB();
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();




