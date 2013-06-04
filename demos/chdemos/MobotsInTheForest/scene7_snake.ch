/* Discription:
 * in this scene, three modules are assembled as a transformative snake.
 *          1st                      2nd                         3rd
 * |---------|--------|       |---------|--------|       |---------|--------|   
 *1|    2    |   3    | 4 X 1 |    2    |   3    | 4 X 1 |    2    |   3    | 4 
 * |---------|--------|       |---------|--------|       |---------|--------|   
 *
 *  NOTE: the surface should be smooth in order for all motions of
 *        this program to work properly. If the surface is not smooth,
 *        you may extract a portion of this program to work.
 */
 
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
CMobot mobot3;

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

mobot1.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
mobot2.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
mobot3.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);

/* First motion */
mobot1.moveJointNB(ROBOT_JOINT4, 90);
mobot2.moveToNB(0, 0, 0, 0);
mobot3.moveJointNB(ROBOT_JOINT1, 90);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();

/* Second motion */
mobot1.moveToNB(0,  -90, 0, 90);
mobot2.moveToNB(0, 0, 0, 0);
mobot3.moveToNB(90, 0, 90,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();

delay(2);

/* third motion */
mobot1.moveToNB(0, 0, 90, 90);
mobot2.moveToNB(0, 0, 0, 0);
mobot3.moveToNB(90, -90, 0,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();

delay(2);

/* fourth motion: spin the middle one */
mobot1.moveJointNB(ROBOT_JOINT4, -90);
mobot2.moveToNB(0, 0, 0, 0);
mobot3.moveJointNB(ROBOT_JOINT1, -90);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();

delay(2);

/* fifth motion */
mobot1.moveToNB(0, 45, 90, 0);
mobot2.moveToNB(0, -45, 45, 0);
mobot3.moveToNB(0, -90, -45,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();

delay(2);

/* sixth motion */
mobot1.moveToNB(45, 45, 90, 45);
mobot2.moveToNB(0, -45, 45, 0);
mobot3.moveToNB(45, -90, -45,  45);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
delay(2);

/* seventh motion */
mobot1.moveToNB(0, 45, 90, 0);
mobot2.moveToNB(0, -45, 45, 0);
mobot3.moveToNB(0, -90, -45,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
delay(2);

/* eighth motion */
mobot1.moveToNB(-45, 45, 90, -45);
mobot2.moveToNB(0, -45, 45, 0);
mobot3.moveToNB(-45, -90, -45,  -45);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
delay(2);

/* ninth motion */
mobot1.moveToNB(0, 45, 90, 0);
mobot2.moveToNB(0, -45, 45, 0);
mobot3.moveToNB(0, -90, -45,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
delay(2);

/* tenth motion */
mobot1.moveToNB(0, 0, 90, 0);
mobot2.moveToNB(0, 0, 0, 0);
mobot3.moveToNB(0, -90, 0,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
delay(2);

/* 11th motion */
mobot2.moveToNB(0, 0, 0, 0);
mobot1.moveJointNB(ROBOT_JOINT4, 90);
mobot3.moveJointNB(ROBOT_JOINT1, 90);
mobot2.moveWait();
mobot1.moveWait();
mobot3.moveWait();
delay(2);

/* 12th motion */
mobot1.moveToNB(0,  -90, 0, 90);
mobot2.moveToNB(0, 0, 0, 0);
mobot3.moveToNB(90, 0, 90,  0);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
delay(2);

/* 13th motion */
mobot1.moveJointNB(ROBOT_JOINT4, -90);
mobot2.moveToNB(0, 0, 0, 0);
mobot3.moveJointNB(ROBOT_JOINT1, -90);
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
