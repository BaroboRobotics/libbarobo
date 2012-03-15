#ifndef _ROBOT_MANAGER_H_
#define _ROBOT_MANAGER_H_

#include "configFile.h"

#define MAX_CONNECTED 100

class RobotManager : public ConfigFile
{
  public:
    RobotManager();
    ~RobotManager();
    bool isConnected(int index);
    void setConnected(int index, bool connected);
    int connect(int index);
    int disconnect(int index);
    int moveUp(int index);
    int moveDown(int index);
    int numConnected();
  private:
    bool _connected[MAX_CONNECTED];
    CMobot *_mobots[MAX_CONNECTED];
    char _connectedAddresses[MAX_CONNECTED][80];
};

#endif
