/* Filename: simple.cpp
 * Rotate the faceplates by 90 degrees */
#include <mobot.h>
#include <stdlib.h>

int main()
{
  int i;
  double a[5], b[5];
  mobot_t comms;
  for(i = 0; i < 5; i++) {
    a[i] = 0;
    b[i] = 0;
  }
  a[1] = 1;
  b[1] = 1;
  Mobot_init(&comms);
  Mobot_connect(&comms);
  if(Mobot_setFourierCoefficients(&comms, MOBOT_JOINT4, a, b)) {
    printf("Error setting coefficients for Mobot %d\n", i);
    exit(-1);
  }
  Mobot_beginFourierControl(&comms, 0xff);
  printf("Press any key to stop.\n");
  getchar();
  Mobot_stop(&comms);
  Mobot_disconnect(&comms);

  return 0;
}
