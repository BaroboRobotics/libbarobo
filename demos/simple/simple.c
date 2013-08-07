#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>

#define NUMBOTS 1

int main()
{
  int i = 0;
  mobot_t mobot;
  Mobot_init(&mobot);
  Mobot_connectWithBluetoothAddress(&mobot, "00:06:66:45:D9:F5", 1);
  Mobot_move(&mobot, 90, 90, 90, 90);

  return 0;
}
