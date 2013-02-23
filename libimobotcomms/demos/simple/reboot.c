#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>

#define NUMBOTS 1

int main()
{
  mobot_t mobot;
  Mobot_init(&mobot);
  Mobot_connectWithTTY(&mobot, "/dev/ttyACM1");
  Mobot_reboot(&mobot);
}
