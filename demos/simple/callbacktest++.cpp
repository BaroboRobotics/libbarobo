#include <mobot.h>
#include <linkbot.h>
#include <stdio.h>
#include <stdlib.h>

#define NUMBOTS 1

void jointcb(int millis, double j1, double j2, double j3, double j4, int mask, void* data)
{
  printf("Joint event %d: %lf %lf %lf %lf %d\n",
      millis, j1, j2, j3, j4, mask);
}

void accelcb(int millis, double x, double y, double z, void* data)
{
  printf("Accel event: %d: %lf %lf %lf\n",
      millis, x, y, z);
}

int main()
{
  int i = 0;
  CLinkbot l;
  l.connectWithAddress("SVZX", 1);
  l.enableJointEventCallback(NULL, jointcb);
  l.enableAccelEventCallback(NULL, accelcb);
  printf("Press enter to quit.\n");
  getchar();

  return 0;
}
