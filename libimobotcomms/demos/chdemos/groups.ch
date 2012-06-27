/* Filename: groups.ch
 * Control multiple Mobot groups simultaneously using the CMobotGroup class */
#include <mobot.h>
CMobot robot1;
CMobot robot2;
CMobot robot3;
CMobot robot4;
CMobotGroup groupA;
CMobotGroup groupB;
CMobotGroup groupC;
CMobotGroup groupD;

/* Connect to the robots listed in the configuration file. */
robot1.connect();
robot2.connect();
robot3.connect();
robot4.connect();

/* Add the robots to our groups. The groups should be organized as such:
 * Group A: 1, 2, 3, 4
 * Group B: 1, 2
 * Group C: 3, 4
 * Group D: 1, 2, 3 */

/* Group A */
groupA.addMobot(robot1);
groupA.addMobot(robot2);
groupA.addMobot(robot3);
groupA.addMobot(robot4);

/* Group B */
groupB.addMobot(robot1);
groupB.addMobot(robot2);

/* Group C */
groupC.addMobot(robot3);
groupC.addMobot(robot4);

/* Group D */
groupD.addMobot(robot1);
groupD.addMobot(robot2);
groupD.addMobot(robot3);

/* Make group B roll forward and group C roll backward at the same time */
groupB.motionRollForwardNB(360);
groupC.motionRollBackwardNB(360);
groupB.motionWait();
groupC.motionWait();

/* Make all the robot stand up */
groupA.motionStand();

/* Make robots 1 and 2 (Group B) rotate counter-clockwise and robots 3 and 4
 * (Group C) rotate clockwise. */
groupB.moveNB(360, 0, 0, 360);
groupC.moveNB(-360, 0, 0, -360);
groupB.moveWait();
groupC.moveWait();

/* Make robot 4 unstand and inchworm while the remaining robots spin. */
groupD.moveContinuousNB(MOBOT_FORWARD, MOBOT_HOLD, MOBOT_HOLD, MOBOT_FORWARD);
robot4.motionUnstand();
robot4.motionInchwormLeft(2);
groupD.motionUnstand();
