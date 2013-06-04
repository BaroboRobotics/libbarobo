#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>

#define NUMBOTS 1

#define ROBOT_ADDR1 0x8D04
#define ROBOT_ADDR2 0xA53F

int main(int argc, char *argv[])
{
  int rc;
  mobot_t mobot;
  if(argc != 3) {
    printf("Expected usage: %s <tty device> <mobot id>\n", argv[0]);
    return 0;
  }
  Mobot_init(&mobot);
  if(rc = Mobot_connectWithTTY(&mobot, argv[1])) {
    fprintf(stderr, "Connection failed with error code %d.\n", rc);
    exit(rc);
  }

  if(rc = Mobot_setID(&mobot, argv[2])) {
    fprintf(stderr, "Error setting the Mobot's ID. %d\n", rc);
    exit(rc);
  }

  return 0;
}
