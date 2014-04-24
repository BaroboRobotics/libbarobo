#include <mobot.h>
#include <linkbot.h>
#include <stdlib.h>
#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#endif

int main()
{
  int i, n, n2;
  double* data, *data2;
  double* time, *time2;
  CLinkbotI mobot;
  CLinkbotI mobot2;
  if(mobot.connectWithAddress("HFRS", 1)) {
	  printf("Error connecting to 59MQ\n");
	  exit(0);
  }
  if(mobot2.connectWithAddress("104C", 1)) {
	  printf("Error connecting to 6P5C\n");
	  exit(0);
  }
  mobot.recordDistanceBegin(ROBOT_JOINT1, 
		  time, 
		  data, 
		  3, 0.1, 1);
  mobot2.recordDistanceBegin(ROBOT_JOINT1, 
		  time2, 
		  data2, 
		  3, 0.1, 1);
  for(i = 0; i < 10; i++) { 
    mobot.moveJointToNB(ROBOT_JOINT1, rand()%90);
    mobot.moveJointToNB(ROBOT_JOINT3, rand()%90);
    mobot2.moveJointToNB(ROBOT_JOINT1, rand()%90);
    mobot2.moveJointToNB(ROBOT_JOINT3, rand()%90);
#ifndef _WIN32
    usleep(500000);
#else
    Sleep(500);
#endif
  }
  mobot.recordDistanceEnd(ROBOT_JOINT1, n);
  mobot2.recordDistanceEnd(ROBOT_JOINT1, n2);
  for(i = 0; i < n; i++ ){
    printf("%lf %lf \n", time[i], data[i]);
  }
  printf("\n\n");
  for(i = 0; i < n2; i++ ){
    printf("%lf %lf \n", time2[i], data2[i]);
  }
  return 0;
}


