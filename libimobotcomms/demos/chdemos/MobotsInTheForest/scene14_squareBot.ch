/* Discription:
 * In this scene, four modules are assembled as a squareBot.
 * Use another computer to control a humanoid robot to dance with squareBot.
 *                          4
 *                       -------
 *                        |   |
 *                        | 2 |
 *                        |   |          ^
 *                       ------- 1st     | forward
 *                        |   |          |
 *                        | 3 |
 *                        |   |
 *                       -------
 *           4th            1             2rd
 *  |---------|--------|  -----  |---------|--------|
 * 4|    3    |    2   |1 |   | 1|    2    |   3    |4
 *  |---------|--------|  -----  |---------|--------|
 *                          1
 *                       -------
 *                        |   |
 *                        | 2 |
 *                        |   |
 *                       ------- 3rd
 *                        |   |
 *                        | 3 | 
 *                        |   |
 *                       -------
 *                          4
 *All switches are on the top of robots.
 *Before assembling, please make sure each robot is in zero position.
 */
 
#include <mobot.h>
CMobot robot1;
CMobot robot2;
CMobot robot3;
CMobot robot4;
int i; 

/* Connect to the robots listed in the configuration file. */
robot1.connect();
robot2.connect();
robot3.connect();
robot4.connect();

robot1.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot2.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot3.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
robot4.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
/*
// Set the robot to "home" position, where all joint angles are 0 degrees.
robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot3.moveToZeroNB();
robot4.moveToZeroNB();
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
robot4.moveWait();

// lift
robot1.moveToNB(0, 0, 90, 0);
robot2.moveToNB(0, 0, 90, 0);
robot3.moveToNB(0, 0, 90, 0);
robot4.moveToNB(0, 0, 90, 0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
robot4.moveWait();
*/
robot1.moveToNB(0, -90, 0, 0);
robot2.moveToNB(0, -90, 0, 0);
robot3.moveToNB(0, -90, 0, 0);
robot4.moveToNB(0, -90, 0, 0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
robot4.moveWait();

robot1.setJointSpeedRatios(0.6, 0.6, 0.6, 0.6);
robot2.setJointSpeedRatios(0.6, 0.6, 0.6, 0.6);
robot3.setJointSpeedRatios(0.6, 0.6, 0.6, 0.6);
robot4.setJointSpeedRatios(0.6, 0.6, 0.6, 0.6);

// move forward two times
for (i = 0; i < 6; i++) {
    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();
    
    robot2.moveToNB(0, 0, 90, 0);
    robot4.moveToNB(0, 0, 90, 0);
    robot2.moveWait();
    robot4.moveWait();
    
    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();

    robot2.moveToNB(90, 0, 90, 0);
    robot4.moveToNB(-90, 0, 90, 0);
    robot2.moveWait();
    robot4.moveWait();

    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();
    
    robot2.moveToNB(90, -90, 0, 0);
    robot4.moveToNB(-90, -90, 0, 0);
    robot2.moveWait();
    robot4.moveWait();

    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();
    
    robot2.moveToNB(0, -90, 0, 0);
    robot4.moveToNB(0, -90, 0, 0);
    robot2.moveWait();
    robot4.moveWait();
}
/*
// move back two times
for (i = 0; i < 2; i++) {
    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();
    
    robot2.moveToNB(0, 0, 90, 0);
    robot4.moveToNB(0, 0, 90, 0);
    robot2.moveWait();
    robot4.moveWait();

    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();
    
    robot2.moveToNB(-90, 0, 90, 0);
    robot4.moveToNB(90, 0, 90, 0);
    robot2.moveWait();
    robot4.moveWait();

    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();
    
    robot2.moveToNB(-90, -90, 0, 0);
    robot4.moveToNB(90, -90, 0, 0);
    robot2.moveWait();
    robot4.moveWait();

    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();
    
    robot2.moveToNB(0, -90, 0, 0);
    robot4.moveToNB(0, -90, 0, 0);
    robot2.moveWait();
    robot4.moveWait();
}*/
// turn right two times
for (i = 0; i < 5; i++) {
    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();
    
    robot2.moveToNB(0, 0, 90, 0);
    robot4.moveToNB(0, 0, 90, 0);
    robot2.moveWait();
    robot4.moveWait();
    
    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();
    
    robot2.moveToNB(-90, 0, 90, 0);
    robot4.moveToNB(-90, 0, 90, 0);
    robot2.moveWait();
    robot4.moveWait();
    
    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();
    
    robot2.moveToNB(-90, -90, 0, 0);
    robot4.moveToNB(-90, -90, 0, 0);
    robot2.moveWait();
    robot4.moveWait();
    
    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();
    
    robot2.moveToNB(0, -90, 0, 0);
    robot4.moveToNB(0, -90, 0, 0);
    robot2.moveWait();
    robot4.moveWait();
}/*
// turn left two times
for (i = 0; i < 2; i++) {
    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();
    
    robot2.moveToNB(0, 0, 90, 0);
    robot4.moveToNB(0, 0, 90, 0);
    robot2.moveWait();
    robot4.moveWait();

    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();
    
    robot2.moveToNB(90, 0, 90, 0);
    robot4.moveToNB(90, 0, 90, 0);
    robot2.moveWait();
    robot4.moveWait();
    
    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();
    
    robot2.moveToNB(90, -90, 0, 0);
    robot4.moveToNB(90, -90, 0, 0);
    robot2.moveWait();
    robot4.moveWait();

    robot1.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot3.moveWait();
    
    robot2.moveToNB(0, -90, 0, 0);
    robot4.moveToNB(0, -90, 0, 0);
    robot2.moveWait();
    robot4.moveWait();
}

robot1.moveToNB(0, 0, 90, 0);
robot2.moveToNB(0, 0, 90, 0);
robot3.moveToNB(0, 0, 90, 0);
robot4.moveToNB(0, 0, 90, 0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
robot4.moveWait();

robot1.moveToNB(0, 0, 0, 0);
robot2.moveToNB(0, 0, 0, 0);
robot3.moveToNB(0, 0, 0, 0);
robot4.moveToNB(0, 0, 0, 0);
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();
robot4.moveWait();
*/
