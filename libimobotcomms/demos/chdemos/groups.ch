/* Filename: groups.ch
 * Control multiple Mobot groups simultaneously using the CMobotGroup class */
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
CMobot mobot3;
CMobot mobot4;
CMobotGroup groupA;
CMobotGroup groupB;
CMobotGroup groupC;
CMobotGroup groupD;

/* Connect to the mobots listed in the configuration file. */
mobot1.connect();
mobot2.connect();
mobot3.connect();
mobot4.connect();

/* Add the mobots to our groups. The groups should be organized as such:
 * Group A: 1, 2, 3, 4
 * Group B: 1, 2
 * Group C: 3, 4
 * Group D: 1, 2, 3 */

/* Group A */
groupA.addMobot(mobot1);
groupA.addMobot(mobot2);
groupA.addMobot(mobot3);
groupA.addMobot(mobot4);

/* Group B */
groupB.addMobot(mobot1);
groupB.addMobot(mobot2);

/* Group C */
groupC.addMobot(mobot3);
groupC.addMobot(mobot4);

/* Group D */
groupD.addMobot(mobot1);
groupD.addMobot(mobot2);
groupD.addMobot(mobot3);

/* Make group B roll forward and group C roll backward at the same time */
groupB.motionRollForwardNB(360);
groupC.motionRollBackwardNB(360);
groupB.motionWait();
groupC.motionWait();

/* Make all the mobot stand up */
groupA.motionStand();

/* Make mobots 1 and 2 (Group B) rotate counter-clockwise and mobots 3 and 4
 * (Group C) rotate clockwise. */
groupB.moveNB(360, 0, 0, 360);
groupC.moveNB(-360, 0, 0, -360);
groupB.moveWait();
groupC.moveWait();

/* Make mobot 4 unstand and inchworm while the remaining mobots spin. */
groupD.moveContinuousNB(MOBOT_FORWARD, MOBOT_HOLD, MOBOT_HOLD, MOBOT_FORWARD);
mobot4.motionUnstand();
mobot4.motionInchwormLeft(2);
groupD.motionUnstand();
