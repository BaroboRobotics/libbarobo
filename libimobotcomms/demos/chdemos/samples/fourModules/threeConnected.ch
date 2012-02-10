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
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
/*single module one */
robot4.motionRollBackward(90);
robot4.motionTurnLeft(90);
robot4.motionRollForward(180);
robot4.motionTurnRight(90);

/* tripleLinked one First motion */
robot1.moveJointNB(ROBOT_JOINT4, 90);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveJointNB(ROBOT_JOINT1, 90);
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
sleep(1);
/* tripleLinked one third motion */
robot1.moveToNB(0, 0, 90, 90);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveToNB(90, -90, 0,  0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
sleep(1);
/* tripleLinked one Fourth motion */
robot1.moveJointNB(ROBOT_JOINT4, -90);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveJointNB(ROBOT_JOINT1, -90);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
sleep(1);
/* tripleLinked one fifth motion */
robot1.moveToNB(0, 45, 90, 0);
robot2.moveToNB(0, -45, 45, 0);
robot3.moveToNB(0, -90, -45,  0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
sleep(10);

/*single module one */
robot4.motionRollForward(360);

/* tenth motion */
robot1.moveToNB(0, 0, 90, 0);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveToNB(0, -90, 0,  0);
robot4.motionTurnRightNB(90);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
robot4.moveWait();
sleep(1);

/* 12th motion */
robot1.moveToNB(0,  -90, 0, 90);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveToNB(90, 0, 90,  0);
robot4.motionRollForwardNB(360);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
robot4.moveWait();
sleep(2);

/* 14th motion */
robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot3.moveToZeroNB();
robot4.motionTurnLeftNB(90);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
robot4.moveWait();


robot4.motionRollBackward(90);
