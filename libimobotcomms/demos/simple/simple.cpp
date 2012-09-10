/* Filename: simple.cpp
 * Rotate the faceplates by 90 degrees */
#include <mobot.h>

int main()
{
  CMobot mobot;

  /* Connect to the paired MoBot */
  mobot.connectWithBluetoothAddress("00:06:66:46:41:FB", 1);
  mobot.move(720, 0, 0, 720);
  mobot.stopAllJoints();

  return 0;
}
