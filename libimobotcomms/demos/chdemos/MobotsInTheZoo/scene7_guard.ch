/* Discription:
 * In this scene, two one-module robots do turn back and forth when they stand as guarding the zoo.
 */
#include <mobot.h>
CMobot robot1;
CMobot robot2;

robot1.connect();
robot2.connect();

// initialization
robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot1.moveWait();
robot1.moveWait();

// stand
robot1.motionStandNB();
robot2.motionStandNB();
robot1.motionWait();
robot2.motionWait();

// turn back and forth
robot1.moveJointNB(MOBOT_JOINT1, 180);
robot2.moveJointNB(MOBOT_JOINT1, 180);
robot1.moveWait();
robot1.moveWait();

robot1.moveJointNB(MOBOT_JOINT1, -180);
robot2.moveJointNB(MOBOT_JOINT1, -180);
robot1.moveWait();
robot1.moveWait();
