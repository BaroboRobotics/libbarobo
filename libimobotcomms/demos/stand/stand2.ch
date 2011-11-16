/* Filename: stand2.ch 
 * Make a MoBot stand up on a faceplate */

#include <mobot.h>

CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

/* Set robot motors to speed of 0.50 */
int i;
for(i = MOBOT_JOINT1; i <= MOBOT_JOINT4; i++) {
  robot.setJointSpeed(i, 0.50);
}
/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

/* Move the robot into a fetal position */
robot.moveJointTo(MOBOT_JOINT2, -85);
robot.moveJointTo(MOBOT_JOINT3, 80);

/* Rotate the bottom faceplate by 45 degrees */
robot.moveJointTo(MOBOT_JOINT1, 45);

/* Lift the body up */
robot.moveJointTo(MOBOT_JOINT2, 20);

/* Pan the robot around for 3 seconds */
robot.setJointSpeed(MOBOT_JOINT1, 0.30);
robot.moveContinuousTime(
    MOBOT_FORWARD,
    MOBOT_NEUTRAL,
    MOBOT_NEUTRAL,
    MOBOT_NEUTRAL,
    3000);

