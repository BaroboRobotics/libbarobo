/* Filename: twoModules.ch
 * Control two modules and make them stand and inchworm simultaneously. */
#include <mobot.h>
CMobot robot1;
CMobot robot2;

/* Connect robot variables to the robot modules. */
robot1.connect();
robot2.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot1.resetToZeroNB();
robot2.resetToZeroNB();

robot1.moveWait();
robot2.moveWait();

/* Instruct the first robot to stand and the second robot to inchworm left four
 * times simultaneously. As soon as the first robot stands up, rotate its joints
 * 1 and 4 360 degrees. */
robot1.motionStandNB();
robot2.motionInchwormLeftNB(4);
robot1.motionWait();
robot1.moveNB(360, 0, 0, 360);
robot1.moveWait();
robot2.motionWait();
/* Instruct the first robot unstand and the second robot inchworm right four
 * times simultaneously. */
robot1.motionUnstandNB();
robot2.motionInchwormRightNB(4);
robot1.motionWait();
robot1.motionTumbleLeft(1);
robot2.motionWait();

