#include <imobot.h>

int main()
{
  CiMobot robot;
  int i;
  for(i = 0; i < 4; i++) {
    robot.setJointSpeed(i, 50);
  }
  robot.initListenerBluetooth(20);
  robot.listenerMainLoop();
  return 0;
}
