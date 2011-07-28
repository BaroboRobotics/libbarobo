#include <imobot.h>

int main()
{
  CiMobot robot;

  robot.initListenerBluetooth(20);
  robot.listenerMainLoop();
  return 0;
}
