#include <mobot.h>
#include <stdlib.h>
#ifndef _WIN32
#include <unistd.h>
#endif

int main()
{
  int i, n, n2;
  double* data, *data2;
  double* time, *time2;
  CMobotI mobot;
  CMobotI mobot2;
  if(mobot.connectWithAddress("HFRS", 1)) {
	  printf("Error connecting to 59MQ\n");
	  exit(0);
  }
  if(mobot2.connectWithAddress("104C", 1)) {
	  printf("Error connecting to 6P5C\n");
	  exit(0);
  }
  mobot.recordDistanceBegin(MOBOT_JOINT1, 
		  time, 
		  data, 
		  3, 0.1, 1);
  mobot2.recordDistanceBegin(MOBOT_JOINT1, 
		  time2, 
		  data2, 
		  3, 0.1, 1);
  for(i = 0; i < 10; i++) { 
    mobot.moveJointToNB(MOBOT_JOINT1, rand()%90);
    mobot.moveJointToNB(MOBOT_JOINT3, rand()%90);
    mobot2.moveJointToNB(MOBOT_JOINT1, rand()%90);
    mobot2.moveJointToNB(MOBOT_JOINT3, rand()%90);
#ifndef _WIN32
    usleep(500000);
#else
    Sleep(500);
#endif
  }
  mobot.recordDistanceEnd(MOBOT_JOINT1, n);
  mobot2.recordDistanceEnd(MOBOT_JOINT1, n2);
  for(i = 0; i < n; i++ ){
    printf("%lf %lf \n", time[i], data[i]);
  }
  printf("\n\n");
  for(i = 0; i < n2; i++ ){
    printf("%lf %lf \n", time2[i], data2[i]);
  }
  return 0;
}


