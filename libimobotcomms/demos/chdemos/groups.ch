/* Filename: groups.ch
 * Control multiple MoBot modules simultaneously using the CMobotGroup class */
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
groupA.addRobot(robot1);
groupA.addRobot(robot2);
groupA.addRobot(robot3);
groupA.addRobot(robot4);

/* Group B */
groupB.addRobot(robot1);
groupB.addRobot(robot2);

/* Group C */
groupC.addRobot(robot1);
groupC.addRobot(robot2);

/* Make all the robot stand up */
groupA.motionStand();

/* Make robots 1 and 2 (Group B) rotate counter-clockwise and robots 3 and 4
 * (Group C) rotate clockwise. */
groupB.moveNB(360, 0, 0, 360);
groupC.moveNB(-360, 0, 0, -360);
groupB.moveWait();
groupC.moveWait();

/* Make robot 4 unstand and inchworm while the remaining robots spin a couple
 * more times */
groupC.moveNB(720, 0, 0, 720);
robot4.motionUnstand();
robot4.motionInchwormLeft(2);
groupC.moveWait();
