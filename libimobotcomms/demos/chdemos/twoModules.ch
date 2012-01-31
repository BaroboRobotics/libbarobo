/* Filename: twoModules.ch
 * Control two modules and make them stand simultaneously. */

#include <mobot.h>

CMobot robot1;
CMobot robot2;

/* Connect robot variables to the robot modules. The */
robot1.connect();
robot2.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot1.moveToZeroNB();
robot2.moveToZeroNB();

robot1.moveWait();
robot2.moveWait();

/* Instruct the first robot to stand and the second robot to inchworm left four
 * times simultaneously. */
robot1.motionStandNB();
robot2.motionInchwormLeftNB();
robot1.motionWait();
robot2.motionWait();
/* Make the first robot unstand and the second robot inchworm right four times
 * simultaneously. */
robot1.motionUnstandNB();
robot2.motionInchwormRightNB(4);
robot1.motionWait();
robot2.motionWait();

