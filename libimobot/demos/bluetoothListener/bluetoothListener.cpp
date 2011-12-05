#include <imobot.h>

int main()
{
  CiMobot robot;
  int bluetoothChannel = 20; // Default channel

  robot.setJointSpeed(MOBOT_JOINT1, 0.50);
  robot.setJointSpeed(MOBOT_JOINT2, 0.50);
  robot.setJointSpeed(MOBOT_JOINT3, 0.50);
  robot.setJointSpeed(MOBOT_JOINT4, 0.50);
  
  robot.initListenerBluetooth(bluetoothChannel);
  robot.listenerMainLoop();

  return 0;
}
