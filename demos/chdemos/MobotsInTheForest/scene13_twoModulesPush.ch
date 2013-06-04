/* Discription:
 * In this scene, two single mobots push the rocket.
 */
 
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;

/* Connect to the paired Mobot */
mobot1.connect();
mobot2.connect();

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot1.resetToZero();
mobot2.resetToZero();

mobot1.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);
mobot2.setJointSpeedRatios(0.4, 0.4, 0.4, 0.4);

mobot1.motionInchwormRightNB(10);
mobot2.motionInchwormRightNB(10);
mobot1.motionWait();
mobot2.motionWait();
