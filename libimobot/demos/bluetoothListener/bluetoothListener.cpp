#include <imobot.h>

int main()
{
  CiMobot robot;
  int i;
  for(i = 1; i <= 4; i++) {
    robot.setJointSpeed((iMobotJointId_t)i, 0.50);
  }
  robot.initListenerBluetooth(20);
  robot.listenerMainLoop();
  return 0;
}
