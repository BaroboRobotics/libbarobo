#ifndef _ROBOT_MANAGER_H_
#define _ROBOT_MANAGER_H_

#include "configFile.h"
#include <mobot.h>

#define MAX_CONNECTED 100

class RobotManager : public ConfigFile
{
  public:
    RobotManager();
    ~RobotManager();
    bool isConnected(int index);
    void setConnected(int index, bool connected);
    int connect(int availableIndex);
    int disconnect(int connectIndex);
    int moveUp(int connectIndex);
    int moveDown(int connectIndex);
    int numConnected();
	const char* getConnected(int connectIndex);
	int availableIndexToIndex(int index);
	int numAvailable();
  private:
    bool _connected[MAX_CONNECTED]; /* Index by ConfigFile */
    CMobot *_mobots[MAX_CONNECTED];
    /* _connectAddresses is an array of pointers to 
       ConfigFile::_addresses */
    char *_connectedAddresses[MAX_CONNECTED];
};

#endif
