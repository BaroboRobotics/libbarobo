/* Discription:
 * In this scene, two one-module mobots do turn back and forth when they stand as guarding the zoo.
 */
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;

mobot1.connect();
mobot2.connect();

// initialization
mobot1.moveToZeroNB();
mobot2.moveToZeroNB();
mobot1.moveWait();
mobot1.moveWait();

// stand
mobot1.motionStandNB();
mobot2.motionStandNB();
mobot1.motionWait();
mobot2.motionWait();

// turn back and forth
mobot1.moveJointNB(MOBOT_JOINT1, 180);
mobot2.moveJointNB(MOBOT_JOINT1, 180);
mobot1.moveWait();
mobot1.moveWait();

mobot1.moveJointNB(MOBOT_JOINT1, -180);
mobot2.moveJointNB(MOBOT_JOINT1, -180);
mobot1.moveWait();
mobot1.moveWait();
