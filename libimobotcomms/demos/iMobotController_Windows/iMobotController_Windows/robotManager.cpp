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

int RobotManager::connect(int availableIndex)
{
  int i;
  int index;
  int err = 0;
  CRecordMobot *mobot = new CRecordMobot;
  if(err = mobot->connectWithAddress( getEntry(index), 1 )) {
    return err;
  }
  /* Insert the newly connected robot to the bottom of the list. */
  _mobots[numConnected()] = mobot;
  _connectedAddresses[numConnected()] = 
	  _addresses[availableIndexToIndex(availableIndex)];
  _connected[availableIndexToIndex(availableIndex)] = true;
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
  _mobots[connectIndex]->disconnect();
  /* Need to shift addresses and mobots up */
  int j;
  for(j = connectIndex+1; j < numConnected(); j++) {
	_connectedAddresses[j-1] = _connectedAddresses[j];
	_mobots[j-1] = _mobots[j];
  }
  _connected[i] = false;
  return 0; 
}

int RobotManager::moveUp(int connectIndex) {
  CRecordMobot* tempMobot;
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
  CRecordMobot* tempMobot;
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

const char* RobotManager::getConnected(int connectIndex) {
	if(connectIndex < 0 || connectIndex >= numConnected()) {
		return NULL;
	}
	return _connectedAddresses[connectIndex];
}

int RobotManager::availableIndexToIndex(int availableIndex)
{
	int index = 0;
	int i;
	if(availableIndex < 0 || availableIndex > numAvailable()) {
		return -1;
	}
	for(index = 0, i = 0; i <= availableIndex; index++) {
		if(_connected[index] == false) {
			i++;
		}
	}
	index--;
	return index;
}

int RobotManager::numAvailable()
{
	return numEntries() - numConnected();
}

CRecordMobot* RobotManager::getMobot(int connectIndex)
{
	if(connectIndex < 0 || connectIndex >= numConnected()) {
		return NULL;
	}
	return _mobots[connectIndex];
}
