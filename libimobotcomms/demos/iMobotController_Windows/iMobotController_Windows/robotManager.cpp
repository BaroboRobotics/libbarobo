#include "robotManager.h"

RobotManager::RobotManager()
{
  int i;
  for(i = 0; i < MAX_CONNECTED; i++) {
    _connected[i] = false;
    _mobots[i] = NULL;
  }
}

RobotManager::~RobotManager()
{
}

bool RobotManager::isConnected(int index) {
  if((index >= numEntries()) || index < 0) {
    return false;
  }
  return _connected[index];
}

void RobotManager::setConnected(int index, bool connected)
{
  _connected[index] = connected;
}

int RobotManager::connect(int index)
{
  int i;
  int err;
  CMobot *mobot = new CMobot;
  if(err = mobot->connectWithAddress( getEntry(index) )) {
    return err;
  }
  _connected[index] = true;
  /* Insert the newly connected robot to the top of the list. First, we must
   * move all of the existing entries down */
  return err;
}
