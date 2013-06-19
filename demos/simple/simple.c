#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>

#define NUMBOTS 1

int main()
{
  int i = 0;
  mobot_t mobot;
  Mobot_init(&mobot);
  Mobot_connect(&mobot);
  Mobot_getBreakoutADC(&mobot, 0, &i);
  printf("%d\n", i);

  return 0;
}
