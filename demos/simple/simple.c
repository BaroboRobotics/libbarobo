#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>

#define NUMBOTS 1

int main()
{
  int i = 0;
  mobot_t mobot;
  Mobot_init(&mobot);
  Mobot_connectWithAddress(&mobot, "P4SP", 1);
  Mobot_move(&mobot, 1, 1, 1, 1);

  return 0;
}
