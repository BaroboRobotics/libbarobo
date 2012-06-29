/* Filename: motion.ch 
   pre-programmed motions */
#include <mobot.h>
CMobot mobot;

mobot.connect();
mobot.moveToZero();

/* set to 50% of the maximum speed */
mobot.setJointSpeedRatios(0.5, 0.5, 0.5, 0.5);

/* pre-programmed motions */
mobot.motionInchwormLeft(2);
mobot.motionInchwormRight(2);
mobot.motionRollForward(360);
mobot.motionRollBackward(360);
mobot.motionTurnRight(360);
mobot.motionTurnLeft(360);

mobot.motionStand(); // stand up
/* Spin the mobot around two revolutions while spinning the top */
mobot.move(2*360, 0, 0, 2*360);
mobot.motionUnstand();
mobot.motionTumbleLeft(2);
