/* Filename: copycat.cpp
 * Connects to the first two mobots in the configuration list. Sets up the
 * mobots so that the second mobot copies the motions of the first mobot. */
#include <mobot.h>

int main()
{
  CMobot mobot1;
  CMobot mobot2;
  double angles[4];
  int i;

  /* Connect to the paired MoBots */
  mobot1.connectWithBluetoothAddress("00:06:66:46:41:FB");
  mobot2.connectWithBluetoothAddress("00:06:66:46:41:FA");
  /* Move both mobots to zero position */
  mobot1.resetToZero();
  mobot2.resetToZero();
  /* Relax both mobots */
  mobot1.stop();
  mobot2.stop();

  while(1) {
    /* Get the beginning time of loop */
    /* Get the first mobots joint angles */
    mobot1.getJointAngles(
        angles[0],
        angles[1],
        angles[2],
        angles[3]);
    /* Move the second mobot */
    mobot2.driveToDirectNB(
        angles[0],
        angles[1],
        angles[2],
        angles[3]);
  }

  return 0;
}
