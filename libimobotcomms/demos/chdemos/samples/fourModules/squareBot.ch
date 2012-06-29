/* Filename: squareBot.ch
 * Control four Mobot modules.
                           4
                        -------
                         |   |
                         | 2 |
                         |   |          ^
                        ------- 1st     | forward
                         |   |          |
                         | 3 |
                         |   |
                        -------
            4th            1             2rd
   |---------|--------|  -----  |---------|--------|
  4|    3    |    2   |1 |   | 1|    2    |   3    |4
   |---------|--------|  -----  |---------|--------|
                           1
                        -------
                         |   |
                         | 2 |
                         |   |
                        ------- 3rd
                         |   |
                         | 3 | 
                         |   |
                        -------
                           4
All switches are on the top of mobots.
Before assembling, please make sure each mobot is in zero position.
*/
#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
CMobot mobot3;
CMobot mobot4;
int i; 

/* Connect to the mobots listed in the configuration file. */
mobot1.connect();
mobot2.connect();
mobot3.connect();
mobot4.connect();

mobot1.setJointSpeedRatios(0.3, 0.3, 0.3, 0.3);
mobot2.setJointSpeedRatios(0.3, 0.3, 0.3, 0.3);
mobot3.setJointSpeedRatios(0.3, 0.3, 0.3, 0.3);
mobot4.setJointSpeedRatios(0.3, 0.3, 0.3, 0.3);

// Set the mobot to "home" position, where all joint angles are 0 degrees.
mobot1.moveToZeroNB();
mobot2.moveToZeroNB();
mobot3.moveToZeroNB();
mobot4.moveToZeroNB();
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
mobot4.moveWait();

// lift
mobot1.moveToNB(0, 0, 90, 0);
mobot2.moveToNB(0, 0, 90, 0);
mobot3.moveToNB(0, 0, 90, 0);
mobot4.moveToNB(0, 0, 90, 0);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
mobot4.moveWait();

mobot1.moveToNB(0, -90, 0, 0);
mobot2.moveToNB(0, -90, 0, 0);
mobot3.moveToNB(0, -90, 0, 0);
mobot4.moveToNB(0, -90, 0, 0);
mobot1.moveWait();
mobot2.moveWait();
mobot3.moveWait();
mobot4.moveWait();

// move forward two times
for (i = 0; i < 2; i++) {
    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(0, 0, 90, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(0, 0, 90, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();

    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(90, 0, 90, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(-90, 0, 90, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();

    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(90, -90, 0, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(-90, -90, 0, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();

    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(0, -90, 0, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(0, -90, 0, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();
}
// move back two times
for (i = 0; i < 2; i++) {
    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(0, 0, 90, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(0, 0, 90, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();

    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(-90, 0, 90, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(90, 0, 90, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();

    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(-90, -90, 0, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(90, -90, 0, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();

    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(0, -90, 0, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(0, -90, 0, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();
}
// turn right two times
for (i = 0; i < 2; i++) {
    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(0, 0, 90, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(0, 0, 90, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();

    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(-90, 0, 90, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(-90, 0, 90, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();

    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(-90, -90, 0, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(-90, -90, 0, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();

    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(0, -90, 0, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(0, -90, 0, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();
}
// turn left two times
for (i = 0; i < 2; i++) {
    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(0, 0, 90, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(0, 0, 90, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();

    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(90, 0, 90, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(90, 0, 90, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();

    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(90, -90, 0, 0);
    mobot3.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(90, -90, 0, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();

    mobot1.moveToNB(0, -90, 0, 0);
    mobot2.moveToNB(0, -90, 0, 0);
    mobot4.moveToNB(0, -90, 0, 0);
    mobot1.moveWait();
    mobot2.moveWait();
    mobot3.moveWait();
    mobot4.moveWait();
}
