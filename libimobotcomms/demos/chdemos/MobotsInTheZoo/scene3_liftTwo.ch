/* Dicription:
 * In this scene, two two-connected mobots do lift.
 *           1st                         2nd
 *  |---------|--------|        |---------|--------|     
 * 1|    2    |    3   | 4  X  1|    2    |   3    | 4
 *  |---------|--------|        |---------|--------|
 *  
 *           3rd                         4th
 *  |---------|--------|        |---------|--------|     
 * 1|    2     |   3   | 4  X  1|    2    |   3    | 4
 *  |---------|--------|        |---------|--------|
 */

#include <mobot.h>
CMobot mobot1;
CMobot mobot2;
CMobot mobot3;
CMobot mobot4;
CMobotGroup group1, group2;

/* Connect mobot variables to the mobot modules. */
mobot1.connect();
mobot2.connect();
mobot3.connect();
mobot4.connect();

/* Add 1st and 3rd  modules as members of one group with the identical motions */
/* Add 2nd and 4th  modules as members of one group with the identical motions */
group1.addRobot(mobot1);
group1.addRobot(mobot3);
group2.addRobot(mobot2);
group2.addRobot(mobot4);

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
group1.resetToZeroNB();
group1.resetToZeroNB();

group1.moveWait();
group2.moveWait();

/* First lift */
group1.moveToNB(0, -90,  0, 0);
group2.moveToNB(0, 0, 90, 0);
group1.moveWait();
group2.moveWait();
/* Second lift */
group1.moveToNB(0, 0, 90,  0);
group2.moveToNB(0,  -90, 0, 0);
group1.moveWait();
group2.moveWait();
/* Back to zero position */
group1.resetToZeroNB();
group2.resetToZeroNB();
group1.moveWait();
group2.moveWait();
