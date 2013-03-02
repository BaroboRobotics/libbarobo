#include <mobot.h>
#include <stdlib.h>
#ifndef _WIN32
#include <unistd.h>
#endif

int main()
{
  int i, n;
  double* data, data2;
  double* time, time2;
  CMobotI mobot;
  CMobotI mobot2;
  mobot.connectWithAddress("59MQ", 1);
  mobot2.connectWithAddress("6P5C", 1);
  mobot.recordDistanceBegin(MOBOT_JOINT1, time, data, 3, 0.1, 0);
  mobot2.recordDistanceBegin(MOBOT_JOINT1, time2, data2, 3, 0.1, 0);
  for(i = 0; i < 10; i++) { 
    mobot.moveJointToNB(MOBOT_JOINT1, rand()%90);
    mobot.moveJointToNB(MOBOT_JOINT3, rand()%90);
#ifndef _WIN32
    usleep(500000);
#else
    Sleep(500);
#endif
  }
  mobot.recordDistanceEnd(MOBOT_JOINT1, n);
  for(i = 0; i < n; i++ ){
    printf("%lf %lf\n", time[i], data[i]);
  }
  return 0;
}


