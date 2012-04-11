/* Filename: gorilla.ch
   Control three modules as a gorilla.
                           1
            2nd         -------         3rd
   |---------|--------|  |   |  |---------|--------|     
  4|    3    |    2   |1 | 2 | 1|    2    |   3    | 4
   |---------|--------|  |   |  |---------|--------|
                        -------
                         |   |
                         | 3 | 1st
                         |   |
                        -------
                           4
All switches are on the top when three modules are assembled.
Before assembling, please make sure every modules are in the home position.
*/
#include <mobot.h>

void initilization(void);
void moveForward(int);
void makeTurn(int);
void standUpSideDown(int);
void show(void);

CMobot robot1;
CMobot robot2;
CMobot robot3;

int turnAngle = 0;

void main(void)
{
    /* Connect robot variables to the robot modules. */
    robot1.connect();
    robot2.connect();
    robot3.connect();

    initilization();
    //standUpSideDown(2);
    show();
    moveForward(2);
    makeTurn(90);
    moveForward(2);
}
void initilization(void)
{
    /* Set the robot to "home" position, where all joint angles are 0 degrees. */
    robot1.moveToZeroNB();
    robot2.moveToZeroNB();
    robot3.moveToZeroNB();
    robot1.moveWait();
    robot2.moveWait();
    robot3.moveWait();
    // move joint 3 of robots to 90 degree
    robot1.moveToNB(0, 0, 90, 0);
    robot2.moveToNB(0, 0, 90, 0);
    robot3.moveToNB(0, 0, 90, 0);
    robot1.moveWait();
    robot2.moveWait();
    robot3.moveWait();

    // move joint 2 of robots to -90 degree
    robot1.moveToNB(0, -90, 0, 0);
    robot2.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot1.moveWait();
    robot2.moveWait();
    robot3.moveWait();
}
void moveForward(int n)
{
    int i;
    robot1.setJointSpeedRatios(0.25, 0.6, 0.6, 0.25);
    robot2.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
    robot3.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);

    /* move forward */
    for (i = 0; i < n; i = i + 1)
    {
        // step A
        robot1.moveToNB(0, -80, -30, turnAngle);
        robot2.moveToNB(40, -90, 0, 0);
        robot3.moveToNB(-40, -90, 0, 0);
        robot1.moveWait();
        robot2.moveWait();
        robot3.moveWait();
        // step B
        robot1.moveToNB(0, -80, -30, turnAngle);
        robot2.moveToNB(-20, -90, 0, 0);
        robot3.moveToNB(20, -90, 0, 0);
        robot1.moveWait();
        robot2.moveWait();
        robot3.moveWait();
        // step C
        robot1.moveToNB(0, -60, 30, turnAngle);
        robot2.moveToNB(0, -90, 0, 0);
        robot3.moveToNB(0, -90, 0, 0);
        robot1.moveWait();
        robot2.moveWait();
        robot3.moveWait();
        // step D
        robot2.moveToNB(-20, -90, 0, 0);
        robot3.moveToNB(20, -90, 0, 0);
        robot2.moveWait();
        robot3.moveWait();
        robot1.moveTo(0, -90, 80, turnAngle);
        robot2.moveToNB(90, -90, 0, 0);
        robot3.moveToNB(-90, -90, 0, 0);
        robot2.moveWait();
        robot3.moveWait();
        robot1.moveTo(0, -90, -90, turnAngle);
        robot1.moveToNB(0, -90, -50, turnAngle);
        robot2.moveToNB(50, -90, 0, 0);
        robot3.moveToNB(-50, -90, 0, 0);
        robot1.moveWait();
        robot2.moveWait();
        robot3.moveWait();
    }
}

void makeTurn(int degree)
{
    // go to the home position
    robot1.moveToNB(0, -80, -40, turnAngle);
    robot2.moveToNB(40, -90, 0, 0);
    robot3.moveToNB(-40, -90, 0, 0);
    robot1.moveWait();
    robot2.moveWait();
    robot3.moveWait();
    turnAngle += degree;
    // make turn
    robot1.moveJointTo(ROBOT_JOINT4, turnAngle);
}


void standUpSideDown(int n)
{
    int i;
    robot1.moveToNB(0, -90, 0, turnAngle);
    robot2.moveToNB(-90, -90, 0, 0);
    robot3.moveToNB(90, -90, 0, 0);
    robot1.moveWait();
    robot2.moveWait();
    robot3.moveWait();

    robot1.moveTo(0, 0, 0, turnAngle);

    for (i = 0; i < n; i = i+1)
    {
        robot2.moveToNB(-90, 0, 90, 0);
        robot3.moveToNB(90, 0, 90, 0);
        robot2.moveWait();
        robot3.moveWait();

        robot2.moveToNB(-90, -90, 0, 0);
        robot3.moveToNB(90, -90, 0, 0);
        robot2.moveWait();
        robot3.moveWait();
    }
    
    robot1.moveTo(0, -90, 0, turnAngle);
    robot1.moveTo(0, -45, 45, turnAngle);
    robot2.moveToNB(0, -90, 0, 0);
    robot3.moveToNB(0, -90, 0, 0);
    robot2.moveWait();
    robot3.moveWait();
}

void show(void)
{
    int i;
    robot1.setJointSpeedRatios(0.15, 0.15, 0.15, 0.3);
    robot2.setJointSpeedRatios(0.15, 0.3, 0.3, 0.15);
    robot3.setJointSpeedRatios(0.15, 0.3, 0.3, 0.15);

    robot1.moveToNB(0, -90, -20, turnAngle);
    robot2.moveToNB(20, -90, 0, 0);
    robot3.moveToNB(-20, -90, 0, 0);
    robot1.moveWait();
    robot2.moveWait();
    robot3.moveWait();

    robot1.moveToNB(0, -90, -30, turnAngle);
    robot2.moveToNB(20, 0, 90, 0);
    robot3.moveToNB(-20, 0, 90, 0);
    robot1.moveWait();
    robot2.moveWait();
    robot3.moveWait();

    robot1.moveToNB(0, -90, -30, turnAngle);
    robot2.moveJointNB(ROBOT_JOINT1, -90);
    robot3.moveJointNB(ROBOT_JOINT1, 90);
    robot1.moveWait();
    robot2.moveWait();
    robot3.moveWait();
    
    robot1.moveToNB(0, -90, -30, turnAngle);
    robot2.moveToNB(-70, -90, 0, 0);
    robot3.moveToNB(70, -90, 0, 0);
    robot1.moveWait();
    robot2.moveWait();
    robot3.moveWait();
    
    for(i = 0; i<1; i = i+1){
    robot2.moveTo(-70, -90, 60, 0);
    robot3.moveTo(70, -90, 60, 0);
    robot2.moveTo(-70, -90, 0, 0);
    robot3.moveTo(70, -90, 0, 0);
    }
    
    robot1.moveToNB(0, -90, -30, turnAngle);
    robot2.moveToNB(-70, 0, 90, 0);
    robot3.moveToNB(70, 0, 90, 0);
    robot1.moveWait();
    robot2.moveWait();
    robot3.moveWait();
    
    robot1.moveToNB(0, -90, -30, turnAngle);
    robot2.moveJointNB(ROBOT_JOINT1, -90);
    robot3.moveJointNB(ROBOT_JOINT1, 90);
    robot1.moveWait();
    robot2.moveWait();
    robot3.moveWait();

    robot2.moveToNB(-160, -90, 0, 0);
    robot3.moveToNB(160, -90, 0, 0);
    robot2.moveWait();
    robot3.moveWait();

    robot2.moveToNB(-160, 0, 90, 0);
    robot3.moveToNB(160, 0, 90, 0);
    robot2.moveWait();
    robot3.moveWait();

    robot1.moveJoint(ROBOT_JOINT4, 360);

    robot2.moveJointNB(ROBOT_JOINT1, 180);
    robot3.moveJointNB(ROBOT_JOINT1, -180);
    robot2.moveWait();
    robot3.moveWait();

    robot2.moveToNB(20, -90, 0, 0);
    robot3.moveToNB(-20, -90, 0, 0);
    robot2.moveWait();
    robot3.moveWait();
}
