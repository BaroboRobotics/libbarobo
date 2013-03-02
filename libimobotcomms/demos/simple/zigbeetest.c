#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>

#define NUMBOTS 1

#define MOBOT_ADDR1 0x8D04
#define MOBOT_ADDR2 0xA53F

int main()
{
  uint8_t channel;
  int i;
  int rc;
  double v;
  int x, y, z;
  double angle1, angle2;
  int power = 255;
  mobot_t mobot;
  mobot_t mobot2;
  mobot_t child;
  Mobot_init(&mobot);
  Mobot_init(&mobot2);
  Mobot_init(&child);
  if(rc = Mobot_connect(&mobot)) {
    fprintf(stderr, "Connection failed.\n");
    exit(0);
  }
  //Mobot_connectWithTTY(&mobot2, "/dev/ttyACM2");
  //printf("0X%2X\n", Mobot_getAddress(&mobot2));
  /*
  Mobot_clearQueriedAddresses(&mobot);
  Mobot_queryAddresses(&mobot);
  sleep(3);
  Mobot_getQueriedAddresses(&mobot);
  */
  /*
  if(rc = Mobot_connectChild(&mobot, &child)) {
  //if(rc = Mobot_connectChildID(&mobot, &child, "A001")) {
    fprintf(stderr, "Error connecting to child.\n");
    exit(0);
  }
  */

  /*
  printf("Setting motor 1 to full power...\n");
  Mobot_setMotorPower(child, 1, 254);
  printf("Setting motor 2 to full power...\n");
  Mobot_setMotorPower(child, 2, 254);
  printf("Done.\n");
  */
#if 0
  double angle;
  /*
  sleep(2);
  Mobot_stop(child);
  return 0;
  */
  for(i = 0; ; i++) {
    Mobot_moveJointToNB(child, 1, i*M_PI/2);
    while(Mobot_isMoving(child)) {
      Mobot_getJointAngle(child, 1, &angle);
      printf("Angle: %lf\n", angle*180/M_PI);
    }
    sleep(2);
  }
#endif
  //Mobot_moveJointToNB(child, 1, M_PI/2.0);
  //Mobot_stop(child);
  /*
    Mobot_setMotorPower(child, 2, 255);
    sleep(5);
    Mobot_setMotorPower(child, 2, 0);
    */
  //Mobot_setJointSpeed(child, 1, 120.0*M_PI/180.0);
  //Mobot_setJointSpeed(child, 2, 120.0*M_PI/180.0);
    //Mobot_setMotorPower(child, 1, 255);
    //Mobot_setMotorPower(child, 2, 255);
    //Mobot_moveJointToNB(child, 2, 0);
    //Mobot_setJointSafetyAngleTimeout(child, 5);
    //Mobot_driveJointToDirect(child, 2, 0 * M_PI/180.0);
  //Mobot_moveJointToNB(child, 1, 0);
  //Mobot_moveJointToNB(child, 2, 0);
  while(1) {
    /*
    power = power * -1;
    Mobot_setMotorPower(child, 1, power);
    Mobot_setMotorPower(child, 2, power);
    */
    //Mobot_setMotorPower(child, 1, rand()%512-255);
    //Mobot_setMotorPower(child, 2, rand()%512-255);
    angle1 = (rand()%360)*M_PI/180.0;
    angle2 = (rand()%360)*M_PI/180.0;
    //printf("Joint Targets: %lf %lf\n", angle1*180.0/M_PI, (-10)*180.0/M_PI);
    //Mobot_moveJointToNB(child, 1, angle1);
    //Mobot_moveJointToNB(child, 2, angle2);
    i = -20;
    for(i = 0; i < 20; i++) {
      //Mobot_getBatteryVoltage(child, &v);
      Mobot_setBuzzerFrequencyOn(&mobot, rand()%2000);
      //Mobot_setBuzzerFrequency(child, 0);
      Mobot_setColorRGB(&mobot, rand()%255, rand()%255, rand()%255);
/*
      if(Mobot_getAccelData(child, &x, &y, &z)) {
        printf("Error getting accel data...\n");
        continue;
      }
      Mobot_getJointAngle(child, 1, &angle1);
      Mobot_getJointAngle(child, 2, &angle2);
      printf("Battery Voltage: %lf\tAccel: %d\t%d\t%d\t Joints: %lf\t%lf\n", 
          v, x, y, z, angle1*180.0/M_PI, angle2*180.0/M_PI);
*/
    }
  }
}
