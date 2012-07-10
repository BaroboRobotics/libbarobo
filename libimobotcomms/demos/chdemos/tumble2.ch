/* Filename: tumble2.ch 
 * Tumbling mobot */
#include <mobot.h>
CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot.resetToZero();

/* Begin tumbling for "num" times */
int i, num = 2;
for(i = 0; i < num; i++) {
    /* First lift and tumble */
    mobot.moveJointTo(MOBOT_JOINT2, -85);
    mobot.moveJointTo(MOBOT_JOINT3, 80);
    mobot.moveJointTo(MOBOT_JOINT2, 0);
    mobot.moveJointTo(MOBOT_JOINT3, 0);
    mobot.moveJointTo(MOBOT_JOINT2, 80);
    mobot.moveJointTo(MOBOT_JOINT2, 45);
    /* Second lift and tumble */
    mobot.moveJointTo(MOBOT_JOINT3, -85);
    mobot.moveJointTo(MOBOT_JOINT2, 80);
    mobot.moveJointTo(MOBOT_JOINT3, 0);
    mobot.moveJointTo(MOBOT_JOINT2, 0);
    mobot.moveJointTo(MOBOT_JOINT3, 80);
    if(i != (num-1)) { /* Do not perform this motion on the last tumble */
        mobot.moveJointTo(MOBOT_JOINT3, 45);
    }
}

/* Unstand the mobot */
mobot.moveJointToNB(MOBOT_JOINT2, 0);
mobot.moveJointToNB(MOBOT_JOINT3, 0);
mobot.moveWait();
mobot.resetToZero();
