/* Filename: voltages.c
 * Prints all encoder voltages in a tabular format */
#include <mobot.h>

int main()
{
  int pins[] = {5, 6, 4, 3, 1, 2};
  int i;
  double voltage;
  br_comms_t* comms = (br_comms_t*)malloc(sizeof(br_comms_t));
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
    Mobot_getButtonVoltage(comms, &voltage);
    printf("BUTTON: %lf", voltage);
    printf("\n");
    sleep(1);
  }

  return 0;
}
