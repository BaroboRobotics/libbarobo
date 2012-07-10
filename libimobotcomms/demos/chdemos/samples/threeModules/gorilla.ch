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

CMobot mobot1;
CMobot mobot2;
CMobot mobot3;

int turnAngle = 0;

void main(void)
{
    /* Connect mobot variables to the mobot modules. */
    mobot1.connect();
    mobot2.connect();
    mobot3.connect();

    initilization();
    //standUpSideDown(2);
    show();
    moveForward(2);
    makeTurn(90);
    moveForward(2);
}
void initilization(void)
{
    /* Set the mobot to "home" position, where all joint angles are 0 degrees. */
    mobot1.resetToZeroNB();
    mobot2.resetToZeroNB();
    mobot3.resetToZeroNB();
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    // move joint 3 of mobots to 90 degree
    mobot1.moveToNB(0, 0, 90, 0);
    mobot2.moveToNB(0, 0, 90, 0);
    mobot3.moveToNB(0, 0, 90, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();

    // move joint 2 of mobots to -90 degree
    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(0, -90, 0, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
}
void moveForward(int n)
{
    int i;
    mobot1.setJointSpeedRatios(0.25, 0.6, 0.6, 0.25);
    mobot2.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);
    mobot3.setJointSpeedRatios(0.25, 0.25, 0.25, 0.25);

    /* move forward */
    for (i = 0; i < n; i = i + 1)
    {
        // step A
        mobot1.moveToNB(0, -80, -30, turnAngle);
        mobot2.moveToNB(40, -90, 0, 0);
        mobot3.moveToNB(-40, -90, 0, 0);
        mobot1.moveWait();
        mobot2.moveWait();
        mobot3.moveWait();
        // step B
        mobot1.moveToNB(0, -80, -30, turnAngle);
        mobot2.moveToNB(-20, -90, 0, 0);
        mobot3.moveToNB(20, -90, 0, 0);
        mobot1.moveWait();
        mobot2.moveWait();
        mobot3.moveWait();
        // step C
        mobot1.moveToNB(0, -60, 30, turnAngle);
        mobot2.moveToNB(0, -90, 0, 0);
        mobot3.moveToNB(0, -90, 0, 0);
        mobot1.moveWait();
        mobot2.moveWait();
        mobot3.moveWait();
        // step D
        mobot2.moveToNB(-20, -90, 0, 0);
        mobot3.moveToNB(20, -90, 0, 0);
        mobot2.moveWait();
        mobot3.moveWait();
        mobot1.moveTo(0, -90, 80, turnAngle);
        mobot2.moveToNB(90, -90, 0, 0);
        mobot3.moveToNB(-90, -90, 0, 0);
        mobot2.moveWait();
        mobot3.moveWait();
        mobot1.moveTo(0, -90, -90, turnAngle);
        mobot1.moveToNB(0, -90, -50, turnAngle);
        mobot2.moveToNB(50, -90, 0, 0);
        mobot3.moveToNB(-50, -90, 0, 0);
        mobot1.moveWait();
        mobot2.moveWait();
        mobot3.moveWait();
    }
}

void makeTurn(int degree)
{
    // go to the home position
    mobot1.moveToNB(0, -80, -40, turnAngle);
    mobot2.moveToNB(40, -90, 0, 0);
    mobot3.moveToNB(-40, -90, 0, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    turnAngle += degree;
    // make turn
    mobot1.moveJointTo(MOBOT_JOINT4, turnAngle);
}


void standUpSideDown(int n)
{
    int i;
    mobot1.moveToNB(0, -90, 0, turnAngle);
    mobot2.moveToNB(-90, -90, 0, 0);
    mobot3.moveToNB(90, -90, 0, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();

    mobot1.moveTo(0, 0, 0, turnAngle);

    for (i = 0; i < n; i = i+1)
    {
        mobot2.moveToNB(-90, 0, 90, 0);
        mobot3.moveToNB(90, 0, 90, 0);
        mobot2.moveWait();
        mobot3.moveWait();

        mobot2.moveToNB(-90, -90, 0, 0);
        mobot3.moveToNB(90, -90, 0, 0);
        mobot2.moveWait();
        mobot3.moveWait();
    }
    
    mobot1.moveTo(0, -90, 0, turnAngle);
    mobot1.moveTo(0, -45, 45, turnAngle);
    mobot2.moveToNB(0, -90, 0, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot2.moveWait();
    mobot3.moveWait();
}

void show(void)
{
    int i;
    mobot1.setJointSpeedRatios(0.15, 0.15, 0.15, 0.3);
    mobot2.setJointSpeedRatios(0.15, 0.3, 0.3, 0.15);
    mobot3.setJointSpeedRatios(0.15, 0.3, 0.3, 0.15);

    mobot1.moveToNB(0, -90, -20, turnAngle);
    mobot2.moveToNB(20, -90, 0, 0);
    mobot3.moveToNB(-20, -90, 0, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();

    mobot1.moveToNB(0, -90, -30, turnAngle);
    mobot2.moveToNB(20, 0, 90, 0);
    mobot3.moveToNB(-20, 0, 90, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();

    mobot1.moveToNB(0, -90, -30, turnAngle);
    mobot2.moveJointNB(MOBOT_JOINT1, -90);
    mobot3.moveJointNB(MOBOT_JOINT1, 90);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    
    mobot1.moveToNB(0, -90, -30, turnAngle);
    mobot2.moveToNB(-70, -90, 0, 0);
    mobot3.moveToNB(70, -90, 0, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    
    for(i = 0; i<1; i = i+1){
    mobot2.moveTo(-70, -90, 60, 0);
    mobot3.moveTo(70, -90, 60, 0);
    mobot2.moveTo(-70, -90, 0, 0);
    mobot3.moveTo(70, -90, 0, 0);
    }
    
    mobot1.moveToNB(0, -90, -30, turnAngle);
    mobot2.moveToNB(-70, 0, 90, 0);
    mobot3.moveToNB(70, 0, 90, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    
    mobot1.moveToNB(0, -90, -30, turnAngle);
    mobot2.moveJointNB(MOBOT_JOINT1, -90);
    mobot3.moveJointNB(MOBOT_JOINT1, 90);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();

    mobot2.moveToNB(-160, -90, 0, 0);
    mobot3.moveToNB(160, -90, 0, 0);
    mobot2.moveWait();
    mobot3.moveWait();

    mobot2.moveToNB(-160, 0, 90, 0);
    mobot3.moveToNB(160, 0, 90, 0);
    mobot2.moveWait();
    mobot3.moveWait();

    mobot1.moveJoint(MOBOT_JOINT4, 360);

    mobot2.moveJointNB(MOBOT_JOINT1, 180);
    mobot3.moveJointNB(MOBOT_JOINT1, -180);
    mobot2.moveWait();
    mobot3.moveWait();

    mobot2.moveToNB(20, -90, 0, 0);
    mobot3.moveToNB(-20, -90, 0, 0);
    mobot2.moveWait();
    mobot3.moveWait();
}
