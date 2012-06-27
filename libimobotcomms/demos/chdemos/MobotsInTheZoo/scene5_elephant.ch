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
 * The robot 5 is the head of the elephant and the robot6 is the tail of the elephant.
 * For the side view, the third robot is behind the first robot ...
 * and the fourth robot is behind the second robot.                        
*/ 
#include <mobot.h>
CMobot robot1;
CMobot robot2;
CMobot robot3;
CMobot robot4;
CMobot robot5;
CMobot robot6;
CMobotGroup group1;
CMobotGroup group2;
int i;

/* Connect to the robots listed in the configuration file. */
robot1.connect();
robot2.connect();
robot3.connect();
robot4.connect();
robot5.connect();
robot6.connect();

/* Add the two modules to be members of our group */
group1.addRobot(robot1);
group1.addRobot(robot4);
group2.addRobot(robot2);
group2.addRobot(robot3);

robot1.setJointSpeedRatios(0.6, 0.6, 0.6, 0.6);
robot2.setJointSpeedRatios(0.6, 0.6, 0.6, 0.6);
robot3.setJointSpeedRatios(0.6, 0.6, 0.6, 0.6);
robot4.setJointSpeedRatios(0.6, 0.6, 0.6, 0.6);

// miove to zero position
group1.moveToZeroNB();
group2.moveToZeroNB();
group1.moveWait();
group2.moveWait();

// move joint 1 to 90 degree
group1.moveJointToNB(MOBOT_JOINT1, 90);
group2.moveJointToNB(MOBOT_JOINT1, 90);
group1.moveWait();
group2.moveWait();
// initilize the head and the tail
robot5.moveToNB(0, 45, -45, 90);
robot6.moveToNB(0, -60, 90, 90);
robot5.moveWait();
robot6.moveWait();

// move the head and the tail
robot5.moveJoint(MOBOT_JOINT4, 90);
robot5.moveJoint(MOBOT_JOINT2, 90);
robot5.moveTo(0, 45, -45, 90);
robot6.moveJoint(MOBOT_JOINT4, 20);
robot6.moveJoint(MOBOT_JOINT4, -40);
robot6.moveJoint(MOBOT_JOINT4, 20);

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
