/* Discription:
 * In this scene, six Mobot modules are assembled as an elephant.
 *      top view              side view
 *                          1            1
                         -------      ------- 
 * -------    -------     |   |        |   |
 * | 3rd |    | 4th |     | 2 |        | 2 | 
 * |     |    |     |     |   |        |   | 
 * -------    -------    ------- 1st ------- 2nd
 * -------    -------     |   |        |   |
 * | 1st |    | 2nd |     | 3 |        | 3 |  
 * |     |    |     |     |   |        |   |
 * -------    -------    -------      ------- 
 *                          4            4
 * The mobot 5 is the head of the elephant and the mobot6 is the tail of the elephant.
 * For the side view, the third mobot is behind the first mobot ...
 * and the fourth mobot is behind the second mobot.                        
*/ 
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
CMobot mobot3;
CMobot mobot4;
CMobot mobot5;
CMobot mobot6;
CMobotGroup group1;
CMobotGroup group2;
int i;

/* Connect to the mobots listed in the configuration file. */
mobot1.connect();
mobot2.connect();
mobot3.connect();
mobot4.connect();
mobot5.connect();
mobot6.connect();

/* Add the two modules to be members of our group */
group1.addRobot(mobot1);
group1.addRobot(mobot4);
group2.addRobot(mobot2);
group2.addRobot(mobot3);

mobot1.setJointSpeedRatios(0.6, 0.6, 0.6, 0.6);
mobot2.setJointSpeedRatios(0.6, 0.6, 0.6, 0.6);
mobot3.setJointSpeedRatios(0.6, 0.6, 0.6, 0.6);
mobot4.setJointSpeedRatios(0.6, 0.6, 0.6, 0.6);

// miove to zero position
group1.resetToZeroNB();
group2.resetToZeroNB();
group1.moveWait();
group2.moveWait();

// move joint 1 to 90 degree
group1.moveJointToNB(MOBOT_JOINT1, 90);
group2.moveJointToNB(MOBOT_JOINT1, 90);
group1.moveWait();
group2.moveWait();
// initilize the head and the tail
mobot5.moveToNB(0, 45, -45, 90);
mobot6.moveToNB(0, -60, 90, 90);
mobot5.moveWait();
mobot6.moveWait();

// move the head and the tail
mobot5.moveJoint(MOBOT_JOINT4, 90);
mobot5.moveJoint(MOBOT_JOINT2, 90);
mobot5.moveTo(0, 45, -45, 90);
mobot6.moveJoint(MOBOT_JOINT4, 20);
mobot6.moveJoint(MOBOT_JOINT4, -40);
mobot6.moveJoint(MOBOT_JOINT4, 20);

// walk forward
for (i = 0; i < 3; i++) {
    // leg 1 and leg 4 move first
    group1.moveTo(90, 30, 30, 0);
    group2.moveTo(90, -30, -30, 0);
    group1.moveTo(90, 0, 0, 0);
    // leg 2 and leg 3 move first
    group2.moveTo(90, 30, 30, 0);
    group1.moveTo(90, -30, -30, 0);
    group2.moveTo(90, 0, 0, 0);
}

// move joint 1 to 90 degree
group1.moveToNB(90, 0, 0, 0);
group2.moveToNB(90, 0, 0, 0);
group1.moveWait();
group2.moveWait();
/*
// walk backward 
for (i = 0; i < 3; i++) {
    // leg 1 and leg 4 move first
    group1.moveTo(90, -30, -30, 0);
    group2.moveTo(90, 30, 30, 0);
    group1.moveTo(90, 0, 0, 0);
    // leg 2 and leg 3 move first
    group2.moveTo(90, -30, -30, 0);
    group1.moveTo(90, 30, 30, 0);
    group2.moveTo(90, 0, 0, 0);
}

// move joint 1 to 90 degree
group1.moveToNB(90, 0, 0, 0);
group2.moveToNB(90, 0, 0, 0);
group1.moveWait();
group2.moveWait();
*/
