/* Filename: twoModules.ch
 * Control two modules and make them stand and inchworm simultaneously. */
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;

/* Connect mobot variables to the mobot modules. */
mobot1.connect();
mobot2.connect();

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot1.moveToZeroNB();
mobot2.moveToZeroNB();

mobot1.moveWait();
mobot2.moveWait();

/* Instruct the first mobot to stand and the second mobot to inchworm left four
 * times simultaneously. As soon as the first mobot stands up, rotate its joints
 * 1 and 4 360 degrees. */
mobot1.motionStandNB();
mobot2.motionInchwormLeftNB(4);
mobot1.motionWait();
mobot1.moveNB(360, 0, 0, 360);
mobot1.moveWait();
mobot2.motionWait();
/* Instruct the first mobot unstand and the second mobot inchworm right four
 * times simultaneously. */
mobot1.motionUnstandNB();
mobot2.motionInchwormRightNB(4);
mobot1.motionWait();
mobot1.motionTumbleLeft(1);
mobot2.motionWait();

