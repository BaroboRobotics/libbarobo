/* Filename: motion.ch 
 * Move the two wheeled mobot. */
#include <mobot.h>
CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot.resetToZero();

mobot.moveTo(90, 0, 0, 90);

mobot.moveTo(180, 0, 0, 180);

mobot.moveTo(0, 0, 0, 0);

/* test all pre-programmed motions */
mobot.motionArch(90);
mobot.motionInchwormLeft(4);
mobot.motionInchwormRight(4);
mobot.motionRollBackward(360);
mobot.motionRollForward(360);
mobot.motionTurnLeft(360);
mobot.motionTurnRight(360);
mobot.motionStand();
mobot.move(360, 0, 0, 360);
mobot.motionUnstand();
mobot.motionTumbleLeft(2);
