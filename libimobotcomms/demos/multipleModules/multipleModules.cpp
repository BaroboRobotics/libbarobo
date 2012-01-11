/* Filename: multipleModules.cpp
 * Control two modules and make them stand simultaneously. */

#include <mobot.h>

int main()
{
  CMobot robot1;
  CMobot robot2;

  /* To connect to multiple modules, each module first needs to be added 
   * to the computer's list of known modules. This process is done using the
   * configuration dialog located within the "Robot -> Configure Robot
   * Bluetooth" menu item. Once multiple modules are configured, each 
   * successive call to the connect() function within the same program 
   * will connect to the next module. */
  robot1.connect();
  robot2.connect();

  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot1.moveToZeroNB();
  robot2.moveToZeroNB();

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
