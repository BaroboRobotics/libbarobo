#include <mobot.h>

int main()
{
  CMobot robot1;
  CMobot robot2;

  /* For multiple robots, we need to use the "connectAddress" function to
   * connect to separate robots. Substitute the string "11:11:11:11:11:11" with
   * the address of the first MoBot and the string "22:22:22:22:22:22" with the
   * string of the second MoBot.*/
  robot1.connectAddress("11:11:11:11:11:11", 20);
  robot2.connectAddress("22:22:22:22:22:22", 20);


  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot1.moveToZero();
  robot2.moveToZero();

  robot1.moveWait();
  robot2.moveWait();

  /* Make both robots stand simultaneously. Note that we must use the
   * non-blocking versions of the motion functions here in order for the robots
   * to perform the motions simultaneously. */
  robot1.motionStandNB();
  robot2.motionStandNB();
  robot1.moveWait();
  robot2.moveWait();

  return 0;
}
