#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>

#define NUMBOTS 1

int main()
{
  mobot_t mobot;
  mobot_t* child;
  int rc;
  mobotMelodyNote_t* head;
  Mobot_init(&mobot);
  //Mobot_connectWithTTY(&mobot, "/dev/ttyACM0");
  if(rc = Mobot_connect(&mobot)) {
    printf("connection failed.\n");
    exit(-1);
  }
  head = Mobot_createMelody(60);
  Mobot_melodyAddNote(head, "C", 16);
  Mobot_melodyAddNote(head, "C", 16);
  Mobot_melodyAddNote(head, "G", 16);
  Mobot_melodyAddNote(head, "G", 16);
  Mobot_melodyAddNote(head, "A", 16);
  Mobot_melodyAddNote(head, "A", 16);
  Mobot_melodyAddNote(head, "G", 8);
  Mobot_melodyAddNote(head, "F", 16);
  Mobot_melodyAddNote(head, "F", 16);
  Mobot_melodyAddNote(head, "E", 16);
  Mobot_melodyAddNote(head, "E", 16);
  Mobot_melodyAddNote(head, "D", 16);
  Mobot_melodyAddNote(head, "D", 16);
  Mobot_melodyAddNote(head, "c", 8);
  /*
  */
  /*
  if(rc = Mobot_connectChildID(&mobot, &child, "004A")) {
    fprintf(stderr, "Error connecting to child.\n");
    //exit(0);
  }
  */
#if 0
  if(rc = Mobot_moveJoint(&mobot, 1, 1)) {
    printf("Error moving joint...\n");
    return 0;
  }
  sleep(3);
  Mobot_stop(&mobot);
#endif
  Mobot_loadMelody(&mobot, 1, head);
  //Mobot_loadMelody(child, 1, head);
  Mobot_playMelody(&mobot, 1);
  //Mobot_playMelody(child, 1);
  /*
  double angle;
  while(1) {
    Mobot_getJointAngle(&mobot, 2, &angle);
    printf("%lf\n", angle);
    sleep(1);
  }
  */
  //Mobot_driveJointToDirectNB(&mobot, 1, 1);
}
