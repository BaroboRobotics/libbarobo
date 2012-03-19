#include "robotManager.h"

RobotManager::RobotManager()
{
  int i;
  for(i = 0; i < MAX_CONNECTED; i++) {
    _connected[i] = false;
    _mobots[i] = NULL;
    _connectedAddresses[i] = NULL;
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
  if(err = mobot->connectWithAddress( getEntry(index), 1 )) {
    return err;
  }
  _connected[index] = true;
  /* Insert the newly connected robot to the top of the list. First, we must
   * move all of the existing entries down */
  for(i = numConnected() - 1; i > 0; i--) {
    _mobots[i+1] = _mobots[i];
    _connectedAddresses[i+1] = _connectedAddresses[i];
  }
  _mobots[0] = mobot;
  _connectedAddresses[0] = _addresses[index];
  return err;
}

int RobotManager::disconnect(int connectIndex)
{
  int i;
  int foundEntry = 0;
  /* Need to find the ConfigFile index of the connected robot */
  for(i = 0; i < numEntries(); i++) {
    if(_connectedAddresses[connectIndex] == _addresses[i]) {
      foundEntry = 1;
      break;
    }
  }
  if(foundEntry == 0) { return -1; }
  _connected[i] = false;
  _mobots[connectIndex]->disconnect();
  return 0; 
}

int RobotManager::moveUp(int connectIndex) {
  CMobot* tempMobot;
  char* tempAddr;
  if(connectIndex < 1 || connectIndex >= numConnected()) return -1;
  /* Swap the robot with the one prior */
  tempMobot = _mobots[connectIndex-1];
  tempAddr = _connectedAddresses[connectIndex-1];
  _mobots[connectIndex-1] = _mobots[connectIndex];
  _connectedAddresses[connectIndex-1] = _connectedAddresses[connectIndex];
  _mobots[connectIndex] = tempMobot;
  _connectedAddresses[connectIndex] = tempAddr;
  return 0;
}

int RobotManager::moveDown(int connectIndex) {
  CMobot* tempMobot;
  char* tempAddr;
  if(connectIndex < 0 || connectIndex >= (numConnected()-1)) return -1;
  tempMobot = _mobots[connectIndex + 1];
  tempAddr = _connectedAddresses[connectIndex + 1];
  _mobots[connectIndex+1] = _mobots[connectIndex];
  _connectedAddresses[connectIndex+1] = _connectedAddresses[connectIndex];
  _mobots[connectIndex] = tempMobot;
  _connectedAddresses[connectIndex] = tempAddr;
  return 0;
}

int RobotManager::numConnected()
{
  int num = 0, i;
  for(i = 0; i < numEntries(); i++) {
    if(_connected[i]) {
      num++;
    }
  }
  return num;
}
