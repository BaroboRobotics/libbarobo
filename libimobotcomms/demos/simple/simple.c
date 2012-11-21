#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>

int main()
{
  mobot_t comms[3];
  int err;
  double a[5], b[5];
  int i, j, joint;
  FILE *fp;
  for(i = 0; i < 5; i++) {
    a[i] = 0;
    b[i] = 0;
  }
  a[1] = 2;
  for(i = 0; i < 3; i++) {
    Mobot_init(&comms[i]);
  }
  for(i = 0; i < 3; i++) {
    err = Mobot_connect(&comms[i]);
    if(err) {
      printf("Connect %d failed with error: %d\n", i, err);
      return 0;
    }
  }
  printf("Connect succeeded. Move to zero.\n");
  for(i = 0; i < 3; i++) {
    Mobot_resetToZero(&comms[i]);
  }
  /* Open the coefficients file */
  fp = fopen("/tmp/fourier_coefs.txt", "r");
  if(fp == NULL) {
    printf("Error opening datafile.\n");
    exit(-1);
  }
  for(i = 0; i < 3; i++) { // For each Mobot
    for(joint = 0; joint < 4; joint++) { // For each Joint
      for(j = 0; j < 5; j++) {
        fscanf(fp, "%lf", &a[j]);
        printf("a: %lf\n", a[j]);
      }
      for(j = 0; j < 5; j++) {
        fscanf(fp, "%lf", &b[j]);
        printf("b: %lf\n", b[j]);
      }
      if(Mobot_setFourierCoefficients(&comms[i], joint+1, a, b)) {
        printf("Error setting coefficients for Mobot %d\n", i);
        exit(-1);
      }
    }
  }
  fclose(fp);
  for(i = 0; i < 3; i++) {
    Mobot_beginFourierControl(&comms[i], 0xff);
  }
  printf("Press any key to stop.\n");
  getchar();
  for(i = 0; i < 3; i++) {
    Mobot_stop(&comms[i]);
    Mobot_disconnect(&comms[i]);
  }
  return 0;
}
