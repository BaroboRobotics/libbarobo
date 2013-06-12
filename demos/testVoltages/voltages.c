/* Filename: voltages.c
 * Prints all encoder voltages in a tabular format */
#include <mobot.h>
#include <stdlib.h>

int main()
{
  int pins[] = {1, 2, 3, 4, 5, 6};
  int i;
  double voltage;
  mobot_t* comms = (mobot_t*)malloc(sizeof(mobot_t));
  Mobot_init(comms);
  if(Mobot_connect(comms)) {
    printf("Error connecting.\n");
    return 0;
  }

  while(1) {
    for(i = 0; i < 6; i++) {
      Mobot_getEncoderVoltage(comms, pins[i], &voltage);
      printf("%lf\t", voltage);
    }
    /*
    Mobot_getButtonVoltage(comms, &voltage);
    printf("BUTTON: %lf", voltage);
    */
    printf("\n");
    usleep(10000);
  }

  return 0;
}
