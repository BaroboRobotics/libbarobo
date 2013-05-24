#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>

#define NUMBOTS 1

int main()
{
  mobot_t mobot;
  mobot_t* child;
  mobotMelodyNote_t* head;
  int rc;
  Mobot_init(&mobot);
  //Mobot_connectWithTTY(&mobot, "/dev/ttyACM0");
  if(rc = Mobot_connectWithTTY(&mobot, "/dev/ttyACM0")) {
    printf("connection failed.\n");
    exit(-1);
  }
  head = Mobot_createMelody(90);
  /*
  Mobot_melodyAddNote(head, "A", 16);
  Mobot_melodyAddNote(head, "C", 16);
  Mobot_melodyAddNote(head, "E", 16);
  Mobot_melodyAddNote(head, "A5", 16);
  Mobot_melodyAddNote(head, "E", 16);
  Mobot_melodyAddNote(head, "C", 16);
  Mobot_melodyAddNote(head, "E", 16);
  Mobot_melodyAddNote(head, "G#", 16);
  Mobot_melodyAddNote(head, "B", 16);
  Mobot_melodyAddNote(head, "E5", 16);
  Mobot_melodyAddNote(head, "B", 16);
  Mobot_melodyAddNote(head, "G#", 16);
  Mobot_melodyAddNote(head, "A", 16);
  Mobot_melodyAddNote(head, "C", 16);
  Mobot_melodyAddNote(head, "E", 16);
  Mobot_melodyAddNote(head, "A5", 16);
  Mobot_melodyAddNote(head, "E", 16);
  Mobot_melodyAddNote(head, "C", 16);
  Mobot_melodyAddNote(head, "A", 16);
  */
  Mobot_melodyAddNote(head, "E5", 16);
  Mobot_melodyAddNote(head, "D#5", 16);
  Mobot_melodyAddNote(head, "E5", 16);
  Mobot_melodyAddNote(head, "D#5", 16);
  Mobot_melodyAddNote(head, "E5", 16);
  Mobot_melodyAddNote(head, "b4", 16);
  Mobot_melodyAddNote(head, "d5", 16);
  Mobot_melodyAddNote(head, "c5", 16);
  Mobot_melodyAddNote(head, "a4", 16);
  Mobot_melodyAddNote(head, "e3", 16);
  Mobot_melodyAddNote(head, "a3", 16);
  Mobot_melodyAddNote(head, "c4", 16);
  Mobot_melodyAddNote(head, "e4", 16);
  Mobot_melodyAddNote(head, "a4", 16);
  Mobot_melodyAddNote(head, "b4", 16);
  Mobot_melodyAddNote(head, "e3", 16);
  Mobot_melodyAddNote(head, "g#3", 16);
  Mobot_melodyAddNote(head, "e4", 16);
  Mobot_melodyAddNote(head, "g#4", 16);
  Mobot_melodyAddNote(head, "b4", 16);
  Mobot_melodyAddNote(head, "c5", 8);
#if 0
  /*
  Mobot_melodyAddNote(head, "e3", 16);
  Mobot_melodyAddNote(head, "a3", 16);
  Mobot_melodyAddNote(head, "e4", 16);
  */
  Mobot_loadMelody(&mobot, 1, head);
  //Mobot_loadMelody(child, 1, head);
  Mobot_playMelody(&mobot, 1);
#ifndef _WIN32
  usleep(2800000);
  usleep(2800000);
#else
  Sleep(3200);
#endif

  head = Mobot_createMelody(120);
#endif
  Mobot_melodyAddNote(head, "E4", 16);
  Mobot_melodyAddNote(head, "E5", 16);
  Mobot_melodyAddNote(head, "D#5", 16);
  Mobot_melodyAddNote(head, "E5", 16);
  Mobot_melodyAddNote(head, "D#5", 16);
  Mobot_melodyAddNote(head, "E5", 16);
  Mobot_melodyAddNote(head, "b4", 16);
  Mobot_melodyAddNote(head, "d5", 16);
  Mobot_melodyAddNote(head, "c5", 16);
  Mobot_melodyAddNote(head, "a4", 16);
  Mobot_melodyAddNote(head, "e3", 16);
  Mobot_melodyAddNote(head, "a3", 16);
  Mobot_melodyAddNote(head, "c4", 16);
  Mobot_melodyAddNote(head, "e4", 16);
  Mobot_melodyAddNote(head, "a4", 16);
  Mobot_melodyAddNote(head, "b4", 16);
  Mobot_melodyAddNote(head, "e3", 16);
  Mobot_melodyAddNote(head, "g#3", 16);
  Mobot_melodyAddNote(head, "d4", 16);
  Mobot_melodyAddNote(head, "c5", 16);
  Mobot_melodyAddNote(head, "b4", 16);
  Mobot_melodyAddNote(head, "a4", 8);
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
