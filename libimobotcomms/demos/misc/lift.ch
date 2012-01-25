/* Filename: lift.ch
   Control two modules and make them stand simultaneously.
   The joint4 of the first robot should be connected to the joint1 of the second robot. */

#include <mobot.h>

CMobot robot1;
CMobot robot2;

/* Connect robot variables to the robot modules. */
robot1.connect();
robot2.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot1.moveToZeroNB();
robot2.moveToZeroNB();

robot1.moveWait();
robot2.moveWait();

/* First lift */
robot1.moveNB(0, deg2rad(-90),  0, 0);
robot2.moveNB(0, 0, deg2rad(90), 0);
robot1.moveWait();
robot2.moveWait();
/* Second lift */
robot1.moveToNB(0, 0, deg2rad(90),  0);
robot2.moveToNB(0,  deg2rad(-90), 0, 0);
robot1.moveWait();
robot2.moveWait();
